#include "sif/motorscootercost.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/nodeinfo.h"
#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/osrm_car_duration.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>

#ifdef INLINE_TEST
#include "test.h"
#include "worker.h"

#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Base transition costs (not toll booth penalties since scooters likely don't take toll roads)
constexpr float kDefaultDestinationOnlyPenalty = 120.0f; // Seconds

// Other options
constexpr float kDefaultUseHills = 0.5f;   // Factor between 0 and 1
constexpr float kDefaultUsePrimary = 0.5f; // Factor between 0 and 1
// Each road class preference defaults to the value of use_primary when the request
// omits it, so a request that only sets use_primary keeps behaving exactly as before.
constexpr float kMultiLaneRightTurnPenalty = 60.0f;
// Signals and stop signs are ubiquitous, so both default to no penalty. A route only
// avoids them when the request explicitly asks for it.
constexpr float kDefaultTrafficSignalPenalty = 0.0f; // Seconds
constexpr float kDefaultStopSignPenalty = 0.0f;      // Seconds
// Cost of turning onto a residential or service road, the same option the truck costing
// already exposes. Defaults to 0 so a route only changes when a request asks for it.
constexpr float kDefaultLowClassPenalty = 0.0f; // Seconds

// Time actually spent waiting at a signalised intersection or a stop sign, added to the
// estimate as well as to the search cost.
//
// A signal is red for roughly half of its cycle, so the expected wait over many crossings
// is about a quarter to a third of a cycle. 15 s reflects a typical urban cycle without
// assuming the rider always stops. A stop sign costs the time to decelerate, look and pull
// away rather than a wait, so it is much smaller.
//
// These are not options. A rider does not get to choose how long a red light lasts, and an
// estimate that ignores it is wrong regardless of how the route was chosen.
constexpr float kTrafficSignalDelay = 15.0f; // Seconds
constexpr float kStopSignDelay = 4.0f;       // Seconds

constexpr uint32_t kMinimumTopSpeed = 20;  // Kilometers per hour
constexpr uint32_t kDefaultTopSpeed = 45;  // Kilometers per hour
constexpr uint32_t kMaximumTopSpeed = 120; // Kilometers per hour
constexpr Surface kMinimumScooterSurface = Surface::kDirt;

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;
constexpr float kTCRamp = 1.5f;
constexpr float kTCRoundabout = 0.5f;

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// Valid ranges and defaults
constexpr ranged_default_t<float> kUseHillsRange{0, kDefaultUseHills, 1.0f};
constexpr ranged_default_t<float> kUsePrimaryRange{0, kDefaultUsePrimary, 1.0f};
constexpr ranged_default_t<uint32_t> kTopSpeedRange{kMinimumTopSpeed, kDefaultTopSpeed,
                                                    kMaximumTopSpeed};
constexpr ranged_default_t<float> kTrafficSignalPenaltyRange{0.0f, kDefaultTrafficSignalPenalty,
                                                            kMaxPenalty};
constexpr ranged_default_t<float> kStopSignPenaltyRange{0.0f, kDefaultStopSignPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kLowClassPenaltyRange{0.0f, kDefaultLowClassPenalty,
                                                        kMaxPenalty};

// Additional penalty to avoid destination only
constexpr float kDestinationOnlyFactor = 0.2f;

// Weighting factor based on road class. These apply penalties to higher class
// roads. These penalties are modulated by the road factor - further
// avoiding higher class roads for those with low propensity for using
// primary roads.
// Floor applied to a road class weight when a request asks to avoid that class.
// Residential is weighted 0.0 and unclassified 0.05, so without this the avoid
// direction would be a no-op for exactly the classes riders most want to steer
// away from on a moped.
constexpr float kMinAvoidableRoadClassFactor = 0.1f;

constexpr float kRoadClassFactor[] = {
    1.0f,  // Motorway
    0.5f,  // Trunk
    0.2f,  // Primary
    0.1f,  // Secondary
    0.05f, // Tertiary
    0.05f, // Unclassified
    0.0f,  // Residential
    0.5f   // Service, other
};

constexpr uint32_t kMaxGradeFactor = 15;

// Avoid hills "strength". How much do we want to avoid a hill. Combines
// with the usehills factor (1.0 - usehills = avoidhills factor) to create
// a weighting penalty per weighted grade factor. This indicates how strongly
// edges with the specified grade are weighted. Note that speed also is
// influenced by grade, so these weights help further avoid hills.
constexpr float kAvoidHillsStrength[] = {
    1.0f,  // -10%  - Very steep downhill
    0.8f,  // -8%
    0.5f,  // -6.5%
    0.2f,  // -5%   - Moderately steep downhill
    0.1f,  // -3%
    0.0f,  // -1.5%
    0.05f, // 0%    - Flat
    0.1f,  // 1.5%
    0.3f,  // 3%
    0.8f,  // 5%
    2.0f,  // 6.5%
    3.0f,  // 8%    - Moderately steep uphill
    4.5f,  // 10%
    6.0f,  // 11.5%
    8.0f,  // 13%
    10.0f  // 15%   - Very steep uphill
};

constexpr float kGradeBasedSpeedFactor[] = {
    1.25f, // -10%  - 45
    1.2f,  // -8%   - 40.5
    1.15f, // -6.5% - 36
    1.1f,  // -5%   - 30.6
    1.05f, // -3%   - 25
    1.0f,  // -1.5% - 21.6
    1.0f,  // 0%    - 18
    1.0f,  // 1.5%  - 17
    0.95f, // 3%    - 15
    0.75f, // 5%    - 13.5
    0.6f,  // 6.5%  - 12
    0.5f,  // 8%    - 10
    0.45f, // 10%   - 9
    0.4f,  // 11.5% - 8
    0.35f, // 13%   - 7
    0.25f  // 15%   - 5.5
};

constexpr float kSurfaceSpeedFactors[] = {1.0f, 1.0f, 0.9f, 0.6f, 0.1f, 0.0f, 0.0f, 0.0f};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.dest_only_penalty_.def = kDefaultDestinationOnlyPenalty;
  cfg.disable_toll_booth_ = true;
  cfg.disable_rail_ferry_ = true;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

bool IsRightTurn(const Turn::Type turntype) {
  return turntype == Turn::Type::kRight || turntype == Turn::Type::kSharpRight;
}

uint32_t EffectiveIngressLaneCount(const graph_tile_ptr& ingress_tile,
                                   const DirectedEdge* ingress_edge) {
  if (!ingress_edge || ingress_edge->internal()) {
    return 0;
  }

  const auto edge_index =
      static_cast<uint32_t>(std::distance(ingress_tile->directededge(0), ingress_edge));
  const auto turn_lane_count =
      ingress_edge->turnlanes() ? ingress_tile->turnlanes(edge_index).size() : 0;
  return std::max<uint32_t>(ingress_edge->lanecount(), turn_lane_count);
}

bool IsPenalizedMultiLaneRightTurn(const bool enabled,
                                   const Turn::Type turntype,
                                   const graph_tile_ptr& ingress_tile,
                                   const DirectedEdge* ingress_edge) {
  return enabled && IsRightTurn(turntype) && ingress_tile &&
         EffectiveIngressLaneCount(ingress_tile, ingress_edge) >= 2;
}

} // namespace

/**
 * Derived class providing dynamic edge costing for "direct" auto routes. This
 * is a route that is generally shortest time but uses route hierarchies that
 * can result in slightly longer routes that avoid shortcuts on residential
 * roads.
 */
class MotorScooterCost : public DynamicCost {
public:
  /**
   * Construct motor scooter costing. Pass in cost type and costing_options using protocol
   * buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  MotorScooterCost(const Costing& costing_options);

  // virtual destructor
  virtual ~MotorScooterCost() {
  }

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const override {
    return true;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  is_dest                     Is a directed edge the destination?
   * @param  pred                        Predecessor edge information.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the directed edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx,
                       uint8_t& destonly_access_restr_mask) const override;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  pred                        Predecessor edge information.
   * @param  opp_edge                    Pointer to the opposing directed edge.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the opposing edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx,
                              uint8_t& destonly_access_restr_mask) const override;

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("MotorScooterCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphId& edgeid,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge          Directed edge (the to edge)
   * @param  node          Node (intersection) where transition occurs.
   * @param  pred          Predecessor edge information.
   * @param  tile          Pointer to the graph tile containing the to edge.
   * @param  reader_getter Functor that facilitates access to a limited version of the graph reader
   * @return Returns the cost and time (seconds)
   */
  virtual Cost
  TransitionCost(const baldr::DirectedEdge* edge,
                 const baldr::NodeInfo* node,
                 const EdgeLabel& pred,
                 const graph_tile_ptr& tile,
                 const std::function<baldr::LimitedGraphReader()>& reader_getter) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx                Directed edge local index
   * @param  node               Node (intersection) where transition occurs.
   * @param  pred               the opposing current edge in the reverse tree.
   * @param  edge               the opposing predecessor in the reverse tree
   * @param  tile               Graphtile that contains the node and the opp_edge
   * @param  edge_id            Graph ID of opp_pred_edge to get its tile if needed
   * @param  reader_getter      Functor that facilitates access to a limited version of the graph
   * reader
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn      Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge,
                                     const graph_tile_ptr& tile,
                                     const GraphId& pred_id,
                                     const std::function<baldr::LimitedGraphReader()>& reader_getter,
                                     const bool has_measured_speed,
                                     const InternalTurn /*internal_turn*/) const override;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override {
    return kSpeedFactor[top_speed_] * min_linear_cost_factor_;
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override {
    return static_cast<uint8_t>(VehicleType::kMotorScooter);
  }
  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by automobile.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (allow_closures || !tile->IsClosed(edge));
  }
  // Hidden in source file so we don't need it to be protected
  // We expose it within the source file for testing purposes
public:
  // Per road class weighting, derived from the use_* preferences. Replaces the single
  // use_primary derived scalar that used to modulate every class at once.
  float road_class_factor_[std::size(kRoadClassFactor)];
  bool avoid_multi_lane_right_turns_;
  float traffic_signal_penalty_; // Seconds added when passing a traffic signal
  float stop_sign_penalty_;      // Seconds added when passing a stop sign
  // Seconds added when turning onto a residential or service road. use_residential weights
  // the distance travelled on such roads; this is paid per entry, so a rider can accept one
  // long back street while still avoiding a chain of short ones.
  float low_class_penalty_;

  // Elevation/grade penalty (weighting applied based on the edge's weighted
  // grade (relative value from 0-15)
  float grade_penalty_[16];
};

// Constructor
MotorScooterCost::MotorScooterCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kMopedAccess),
      avoid_multi_lane_right_turns_(costing.options().avoid_multi_lane_right_turns()),
      traffic_signal_penalty_(costing.options().traffic_signal_penalty()),
      stop_sign_penalty_(costing.options().stop_sign_penalty()),
      low_class_penalty_(costing.options().low_class_penalty()) {
  const auto& costing_options = costing.options();

  // Get the base costs
  get_base_costs(costing);

  // Set grade penalties based on use_hills option.
  // Scale from 0 (avoid hills) to 1 (don't avoid hills)
  float use_hills = costing_options.use_hills();
  float avoid_hills = (1.0f - use_hills);
  for (uint32_t i = 0; i <= kMaxGradeFactor; ++i) {
    grade_penalty_[i] = avoid_hills * kAvoidHillsStrength[i];
  }

  // Set the road classification factors from the use_* options. Each scales from
  // 0 (avoid this class) to 1 (don't avoid it). Above 0.5 the weight difference
  // between road classes shrinks; below 0.5 it grows.
  //
  // motor_scooter used to expose use_primary alone and apply the scalar it derived
  // to every class, so "prefer minor roads" could not be expressed at all. Each
  // class now has its own preference, and any class the request leaves out falls
  // back to use_primary, which keeps an existing use_primary-only request identical.
  const float use_primary = costing_options.use_primary();
  // Motorway, trunk and primary follow use_primary. The remaining classes follow their own
  // preference when the request named one and fall back to use_primary otherwise, which is
  // what keeps a use_primary-only request identical to the behaviour before these options
  // existed.
  struct ClassPreference {
    float use_value;
    bool named;
  };
  const ClassPreference preference_by_class[std::size(kRoadClassFactor)] = {
      {use_primary, false},
      {use_primary, false},
      {use_primary, false},
      {costing_options.has_use_secondary() ? costing_options.use_secondary() : use_primary,
       costing_options.has_use_secondary()},
      {costing_options.has_use_tertiary() ? costing_options.use_tertiary() : use_primary,
       costing_options.has_use_tertiary()},
      {costing_options.has_use_unclassified() ? costing_options.use_unclassified() : use_primary,
       costing_options.has_use_unclassified()},
      {costing_options.has_use_residential() ? costing_options.use_residential() : use_primary,
       costing_options.has_use_residential()},
      {costing_options.has_use_service() ? costing_options.use_service() : use_primary,
       costing_options.has_use_service()},
  };

  for (size_t i = 0; i < std::size(kRoadClassFactor); ++i) {
    const float use_value = preference_by_class[i].use_value;
    const float scale = (use_value >= 0.5f) ? 1.5f - use_value : 3.0f - use_value * 5.0f;

    // Residential is weighted 0.0 and unclassified 0.05, so scaling alone can never express
    // avoiding them - zero stays zero however hard the scale pushes. Raise a floor for those
    // classes as the preference moves below 0.5, ramping in from nothing at 0.5 to the full
    // floor at 0 so there is no step in cost as a caller sweeps the value.
    //
    // The floor applies only to a class the request named. A class that merely inherited
    // use_primary keeps its original weight, so use_primary on its own still reproduces the
    // old numbers exactly, including for use_primary below 0.5.
    float weight = kRoadClassFactor[i];
    if (preference_by_class[i].named && use_value < 0.5f) {
      const float ramp = 1.0f - use_value * 2.0f;
      weight = std::max(weight, kMinAvoidableRoadClassFactor * ramp);
    }
    road_class_factor_[i] = scale * weight;
  }
}

// Check if access is allowed on the specified edge.
bool MotorScooterCost::Allowed(const baldr::DirectedEdge* edge,
                               const bool is_dest,
                               const EdgeLabel& pred,
                               const graph_tile_ptr& tile,
                               const baldr::GraphId& edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               uint8_t& restriction_idx,
                               uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && !ignore_turn_restrictions_) ||
      (edge->surface() > kMinimumScooterSurface) || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) || CheckExclusions(edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx, destonly_access_restr_mask);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool MotorScooterCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                      const EdgeLabel& pred,
                                      const baldr::DirectedEdge* opp_edge,
                                      const graph_tile_ptr& tile,
                                      const baldr::GraphId& opp_edgeid,
                                      const uint64_t current_time,
                                      const uint32_t tz_index,
                                      uint8_t& restriction_idx,
                                      uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  // Allow U-turns at dead-end nodes.
  if (!IsAccessible(opp_edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) && !ignore_turn_restrictions_) ||
      (opp_edge->surface() > kMinimumScooterSurface) || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) || CheckExclusions(opp_edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, opp_edge, false, tile, opp_edgeid,
                                           current_time, tz_index, restriction_idx,
                                           destonly_access_restr_mask);
}

Cost MotorScooterCost::EdgeCost(const baldr::DirectedEdge* edge,
                                const baldr::GraphId& edgeid,
                                const graph_tile_ptr& tile,
                                const baldr::TimeInfo& time_info,
                                uint8_t& flow_sources) const {
  auto speed = fixed_speed_ == baldr::kDisableFixedSpeed
                   ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, false, &flow_sources,
                                    time_info.seconds_from_now)
                   : fixed_speed_;

  if (edge->use() == Use::kFerry) {
    assert(speed < kSpeedFactor.size());
    float sec = (edge->length() * kSpeedFactor[speed]);
    return {sec * ferry_factor_, sec};
  }

  // prevent scooter speed to become 0
  uint32_t scooter_speed =
      std::max(1.f, (std::min(top_speed_, speed) *
                     kSurfaceSpeedFactors[static_cast<uint32_t>(edge->surface())] *
                     kGradeBasedSpeedFactor[static_cast<uint32_t>(edge->weighted_grade())]));

  assert(scooter_speed < kSpeedFactor.size());
  float sec = (edge->length() * kSpeedFactor[scooter_speed]);

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  float factor = 1.0f + (kDensityFactor[edge->density()] - 0.85f) +
                 road_class_factor_[static_cast<uint32_t>(edge->classification())] +
                 grade_penalty_[static_cast<uint32_t>(edge->weighted_grade())] +
                 SpeedPenalty(edge, tile, time_info, flow_sources, speed);

  if (edge->destonly()) {
    factor += kDestinationOnlyFactor;
  }

  if (edge->use() == Use::kTrack) {
    factor *= track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= service_factor_;
  }
  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }

  factor *= EdgeFactor(edgeid);
  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost MotorScooterCost::TransitionCost(
    const baldr::DirectedEdge* edge,
    const baldr::NodeInfo* node,
    const EdgeLabel& pred,
    const graph_tile_ptr& /*tile*/,
    const std::function<baldr::LimitedGraphReader()>& reader_getter) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, idx);

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred.use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred.use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;

    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;

    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, false, InternalTurn::kNoTurn,
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!pred.has_measured_speed()) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  if (low_class_penalty_ > 0.0f && (edge->classification() == baldr::RoadClass::kResidential ||
                                    edge->classification() == baldr::RoadClass::kServiceOther)) {
    // This is a generalized search cost only; do not alter ETA.
    c.cost += low_class_penalty_;
  }
  if (node->traffic_signal()) {
    // The wait is time the rider actually spends, so it belongs in the estimate. The
    // penalty on top of it only steers the search and leaves the estimate alone.
    c.secs += kTrafficSignalDelay;
    c.cost += kTrafficSignalDelay + traffic_signal_penalty_;
  }

  // Both the multi-lane right turn check and the stop sign live on the edge we are leaving,
  // so only reach for the graph reader once and only when one of them is requested.
  // The stop sign is now always worth looking up: even without a penalty it costs the rider
  // time, so the estimate needs it.
  const bool needs_ingress_edge = true;
  if (needs_ingress_edge) {
    auto reader = reader_getter();
    const auto ingress_tile = reader.GetGraphTile(pred.edgeid());
    const auto* ingress_edge = ingress_tile ? ingress_tile->directededge(pred.edgeid()) : nullptr;
    if (IsPenalizedMultiLaneRightTurn(avoid_multi_lane_right_turns_, turntype, ingress_tile,
                                      ingress_edge)) {
      // This is a generalized search cost only; do not alter ETA.
      c.cost += kMultiLaneRightTurnPenalty;
    }
    if (ingress_edge && ingress_edge->stop_sign()) {
      c.secs += kStopSignDelay;
      c.cost += kStopSignDelay + stop_sign_penalty_;
    }
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost MotorScooterCost::TransitionCostReverse(
    const uint32_t idx,
    const baldr::NodeInfo* node,
    const baldr::DirectedEdge* ingress_edge,
    const baldr::DirectedEdge* outgoing_edge,
    const graph_tile_ptr& ingress_tile,
    const GraphId& /*pred_id*/,
    const std::function<baldr::LimitedGraphReader()>& /*reader_getter*/,
    const bool has_measured_speed,
    const InternalTurn /*internal_turn*/) const {

  // MotorScooters should be able to make uturns on short internal edges; therefore, InternalTurn
  // is ignored for now.
  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, outgoing_edge, ingress_edge, idx);
  c.secs += OSRMCarTurnDuration(outgoing_edge, node, ingress_edge->opp_local_idx());

  const auto stopimpact = outgoing_edge->stopimpact(idx);
  const auto turntype = outgoing_edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (outgoing_edge->edge_to_right(idx) && outgoing_edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((outgoing_edge->use() != Use::kRamp && ingress_edge->use() == Use::kRamp) ||
        (outgoing_edge->use() == Use::kRamp && ingress_edge->use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (outgoing_edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;
    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;
    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, outgoing_edge, has_reverse, has_left, has_right, false,
                    InternalTurn::kNoTurn, seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }
  if (IsPenalizedMultiLaneRightTurn(avoid_multi_lane_right_turns_, turntype, ingress_tile,
                                    ingress_edge)) {
    // This is a generalized search cost only; do not alter ETA.
    c.cost += kMultiLaneRightTurnPenalty;
  }
  if (low_class_penalty_ > 0.0f &&
      (outgoing_edge->classification() == baldr::RoadClass::kResidential ||
       outgoing_edge->classification() == baldr::RoadClass::kServiceOther)) {
    // This is a generalized search cost only; do not alter ETA.
    c.cost += low_class_penalty_;
  }
  if (node->traffic_signal()) {
    c.secs += kTrafficSignalDelay;
    c.cost += kTrafficSignalDelay + traffic_signal_penalty_;
  }
  if (ingress_edge && ingress_edge->stop_sign()) {
    c.secs += kStopSignDelay;
    c.cost += kStopSignDelay + stop_sign_penalty_;
  }
  return c;
}

void ParseMotorScooterCostOptions(const rapidjson::Document& doc,
                                  const std::string& costing_options_key,
                                  Costing* c,
                                  google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  c->set_type(Costing::motor_scooter);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseHillsRange, json, "/use_hills", use_hills, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUsePrimaryRange, json, "/use_primary", use_primary, warnings);

  // Only set a road class preference the request actually named. JSON_PBF_RANGED_DEFAULT
  // always calls the setter, which would leave every presence bit set and make an
  // omitted class indistinguishable from an explicit one - and the two are treated
  // differently below, so that distinction has to survive parsing.
  const auto parse_class = [&](const char* key, void (Costing::Options::*setter)(float)) {
    if (auto value = rapidjson::get_optional<float>(json, key)) {
      bool clamped = false;
      (co->*setter)(kUsePrimaryRange(*value, clamped));
      if (clamped) {
        auto warning = warnings.Add();
        warning->set_description("'" + std::string(key) + "' has been clamped.");
        warning->set_code(400);
      }
    }
  };
  parse_class("/use_secondary", &Costing::Options::set_use_secondary);
  parse_class("/use_tertiary", &Costing::Options::set_use_tertiary);
  parse_class("/use_unclassified", &Costing::Options::set_use_unclassified);
  parse_class("/use_residential", &Costing::Options::set_use_residential);
  parse_class("/use_service", &Costing::Options::set_use_service);
  JSON_PBF_DEFAULT_V2(co, false, json, "/avoid_multi_lane_right_turns", avoid_multi_lane_right_turns);
  JSON_PBF_RANGED_DEFAULT(co, kTrafficSignalPenaltyRange, json, "/traffic_signal_penalty",
                          traffic_signal_penalty, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kStopSignPenaltyRange, json, "/stop_sign_penalty", stop_sign_penalty,
                          warnings);
  JSON_PBF_RANGED_DEFAULT(co, kLowClassPenaltyRange, json, "/low_class_penalty",
                          low_class_penalty, warnings);
}

cost_ptr_t CreateMotorScooterCost(const Costing& costing_options) {
  return std::make_shared<MotorScooterCost>(costing_options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestMotorScooterCost : public MotorScooterCost {
public:
  TestMotorScooterCost(const Costing& costing_options) : MotorScooterCost(costing_options) {};

  using MotorScooterCost::alley_penalty_;
  using MotorScooterCost::country_crossing_cost_;
  using MotorScooterCost::destination_only_penalty_;
  using MotorScooterCost::ferry_transition_cost_;
  using MotorScooterCost::gate_cost_;
  using MotorScooterCost::maneuver_penalty_;
  using MotorScooterCost::service_factor_;
  using MotorScooterCost::service_penalty_;
  using MotorScooterCost::low_class_penalty_;
  using MotorScooterCost::road_class_factor_;
  using MotorScooterCost::stop_sign_penalty_;
  using MotorScooterCost::top_speed_;
  using MotorScooterCost::traffic_signal_penalty_;
};

TestMotorScooterCost* make_motorscootercost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "motor_scooter", "costing_options":{"motor_scooter":{")" << property << R"(":)"
     << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestMotorScooterCost(request.options().costings().find(Costing::motor_scooter)->second);
}

template <typename T>
std::uniform_real_distribution<T>*
make_real_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

template <typename T>
std::uniform_int_distribution<T>* make_int_distributor_from_range(const ranged_default_t<T>& range) {
  T rangeLength = range.max - range.min;
  return new std::uniform_int_distribution<T>(range.min - rangeLength, range.max + rangeLength);
}

TEST(MotorscooterCost, testTrafficSignalPenaltyDefaultsToZero) {
  Api request;
  ParseApi(R"({"costing":"motor_scooter"})", valhalla::Options::route, request);
  TestMotorScooterCost cost(request.options().costings().find(Costing::motor_scooter)->second);

  // Signals are everywhere, so the default must not change existing routes.
  EXPECT_EQ(cost.traffic_signal_penalty_, kDefaultTrafficSignalPenalty);
  EXPECT_EQ(cost.stop_sign_penalty_, kDefaultStopSignPenalty);
}

TEST(MotorscooterCost, testTrafficSignalPenaltyIsParsed) {
  Api request;
  ParseApi(
      R"({"costing":"motor_scooter","costing_options":{"motor_scooter":{"traffic_signal_penalty":45.5,"stop_sign_penalty":12.25}}})",
      valhalla::Options::route, request);
  TestMotorScooterCost cost(request.options().costings().find(Costing::motor_scooter)->second);

  EXPECT_FLOAT_EQ(cost.traffic_signal_penalty_, 45.5f);
  EXPECT_FLOAT_EQ(cost.stop_sign_penalty_, 12.25f);
}

namespace {
// The weighting motor_scooter applied before per-class preferences existed: one scalar
// from use_primary multiplied into every class weight.
float LegacyRoadClassFactor(float use_primary, size_t road_class) {
  const float scale = (use_primary >= 0.5f) ? 1.5f - use_primary : 3.0f - use_primary * 5.0f;
  return scale * kRoadClassFactor[road_class];
}

TestMotorScooterCost* make_cost(const std::string& costing_options_json) {
  Api request;
  ParseApi(R"({"costing":"motor_scooter","costing_options":{"motor_scooter":)" +
               costing_options_json + "}}",
           valhalla::Options::route, request);
  return new TestMotorScooterCost(
      request.options().costings().find(Costing::motor_scooter)->second);
}
} // namespace

TEST(MotorscooterCost, testUsePrimaryAloneKeepsLegacyWeights) {
  // The property that matters most: a request naming only use_primary must weight every
  // class exactly as it did before the per-class options existed. 0.2 is included because
  // it drives the classes whose weight is 0.0 or 0.05, where an avoid floor would show up.
  for (float use_primary : {0.0f, 0.2f, 0.5f, 0.9f, 1.0f}) {
    std::shared_ptr<TestMotorScooterCost> cost(
        make_cost(R"({"use_primary":)" + std::to_string(use_primary) + "}"));
    for (size_t i = 0; i < std::size(kRoadClassFactor); ++i) {
      EXPECT_FLOAT_EQ(cost->road_class_factor_[i], LegacyRoadClassFactor(use_primary, i))
          << "use_primary=" << use_primary << " class=" << i;
    }
  }
}

TEST(MotorscooterCost, testDefaultRequestKeepsLegacyWeights) {
  Api request;
  ParseApi(R"({"costing":"motor_scooter"})", valhalla::Options::route, request);
  TestMotorScooterCost cost(request.options().costings().find(Costing::motor_scooter)->second);

  for (size_t i = 0; i < std::size(kRoadClassFactor); ++i) {
    EXPECT_FLOAT_EQ(cost.road_class_factor_[i], LegacyRoadClassFactor(kDefaultUsePrimary, i));
  }
}

TEST(MotorscooterCost, testNamingOneClassLeavesTheOthersOnUsePrimary) {
  std::shared_ptr<TestMotorScooterCost> cost(
      make_cost(R"({"use_primary":0.2,"use_residential":1.0})"));

  const auto residential = static_cast<size_t>(baldr::RoadClass::kResidential);
  for (size_t i = 0; i < std::size(kRoadClassFactor); ++i) {
    if (i == residential) {
      continue;
    }
    EXPECT_FLOAT_EQ(cost->road_class_factor_[i], LegacyRoadClassFactor(0.2f, i)) << "class=" << i;
  }
  // Residential is weighted 0.0, so preferring it cannot move the weight off zero.
  EXPECT_FLOAT_EQ(cost->road_class_factor_[residential], 0.0f);
}

TEST(MotorscooterCost, testAvoidingAZeroWeightedClassIsExpressible) {
  const auto residential = static_cast<size_t>(baldr::RoadClass::kResidential);

  std::shared_ptr<TestMotorScooterCost> neutral(make_cost(R"({"use_residential":0.5})"));
  EXPECT_FLOAT_EQ(neutral->road_class_factor_[residential], 0.0f)
      << "0.5 is the neutral point and must not raise the floor";

  std::shared_ptr<TestMotorScooterCost> avoiding(make_cost(R"({"use_residential":0.0})"));
  EXPECT_GT(avoiding->road_class_factor_[residential], 0.0f);
}

TEST(MotorscooterCost, testAvoidFloorRampsInWithoutAStep) {
  // Sweeping the preference across 0.5 must not jump: the floor ramps in from nothing.
  const auto residential = static_cast<size_t>(baldr::RoadClass::kResidential);
  std::shared_ptr<TestMotorScooterCost> just_above(make_cost(R"({"use_residential":0.51})"));
  std::shared_ptr<TestMotorScooterCost> just_below(make_cost(R"({"use_residential":0.49})"));

  const float step = std::abs(just_below->road_class_factor_[residential] -
                              just_above->road_class_factor_[residential]);
  EXPECT_LT(step, 0.01f) << "crossing 0.5 must be continuous";
}

TEST(MotorscooterCost, testSignalAndStopDelaysAreNotOptions) {
  // The wait at a red light is not something a request opts into: it is time the rider
  // spends whatever the routing preferences say. Guard the magnitudes so a later change
  // has to be deliberate.
  EXPECT_GT(kTrafficSignalDelay, 0.0f);
  EXPECT_GT(kStopSignDelay, 0.0f);
  EXPECT_GT(kTrafficSignalDelay, kStopSignDelay)
      << "waiting for a light takes longer than pulling away from a stop sign";
  EXPECT_LE(kTrafficSignalDelay, 30.0f) << "a full cycle would over-estimate every crossing";
}

TEST(MotorscooterCost, testLowClassPenaltyDefaultsToZeroAndParses) {
  Api defaults;
  ParseApi(R"({"costing":"motor_scooter"})", valhalla::Options::route, defaults);
  TestMotorScooterCost unset(defaults.options().costings().find(Costing::motor_scooter)->second);
  EXPECT_EQ(unset.low_class_penalty_, kDefaultLowClassPenalty);

  std::shared_ptr<TestMotorScooterCost> set(make_cost(R"({"low_class_penalty":45})"));
  EXPECT_FLOAT_EQ(set->low_class_penalty_, 45.0f);
}

TEST(MotorscooterCost, testMultiLaneRightTurnOption) {
  Api disabled_request;
  ParseApi(R"({"costing":"motor_scooter"})", valhalla::Options::route, disabled_request);
  TestMotorScooterCost disabled(
      disabled_request.options().costings().find(Costing::motor_scooter)->second);
  EXPECT_FALSE(disabled.avoid_multi_lane_right_turns_);

  Api enabled_request;
  ParseApi(
      R"({"costing":"motor_scooter","costing_options":{"motor_scooter":{"avoid_multi_lane_right_turns":true}}})",
      valhalla::Options::route, enabled_request);
  TestMotorScooterCost enabled(
      enabled_request.options().costings().find(Costing::motor_scooter)->second);
  EXPECT_TRUE(enabled.avoid_multi_lane_right_turns_);

  EXPECT_TRUE(IsRightTurn(Turn::Type::kRight));
  EXPECT_TRUE(IsRightTurn(Turn::Type::kSharpRight));
  EXPECT_FALSE(IsRightTurn(Turn::Type::kStraight));
  EXPECT_FALSE(IsRightTurn(Turn::Type::kLeft));
  EXPECT_FALSE(IsRightTurn(Turn::Type::kSharpLeft));
  EXPECT_FALSE(IsRightTurn(Turn::Type::kReverse));
}

TEST(MotorscooterCost, testMotorScooterCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> fDistributor;
  std::shared_ptr<std::uniform_int_distribution<uint32_t>> iDistributor;
  std::shared_ptr<TestMotorScooterCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("maneuver_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("alley_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("destination_only_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  fDistributor.reset(make_real_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("gate_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // country_crossing_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("country_crossing_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  fDistributor.reset(make_real_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("country_crossing_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_cost_ (Cost.secs)
  fDistributor.reset(make_real_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("ferry_cost", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // top_speed_
  iDistributor.reset(make_int_distributor_from_range(kTopSpeedRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("top_speed", (*iDistributor)(generator)));
    EXPECT_THAT(ctorTester->top_speed_, test::IsBetween(kTopSpeedRange.min, kTopSpeedRange.max));
  }

  // traffic_signal_penalty_
  fDistributor.reset(make_real_distributor_from_range(kTrafficSignalPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("traffic_signal_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->traffic_signal_penalty_,
                test::IsBetween(kTrafficSignalPenaltyRange.min, kTrafficSignalPenaltyRange.max));
  }

  // stop_sign_penalty_
  fDistributor.reset(make_real_distributor_from_range(kStopSignPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_motorscootercost_from_json("stop_sign_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->stop_sign_penalty_,
                test::IsBetween(kStopSignPenaltyRange.min, kStopSignPenaltyRange.max));
  }

  // service_penalty_
  fDistributor.reset(make_real_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("service_penalty", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // service_factor_
  fDistributor.reset(make_real_distributor_from_range(defaults.service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("service_factor", (*fDistributor)(generator)));
    EXPECT_THAT(ctorTester->service_factor_,
                test::IsBetween(defaults.service_factor_.min, defaults.service_factor_.max));
  }

  /**
  // use_ferry
  fDistributor.reset(make_real_distributor_from_range(defaults.use_ferry_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_ferry", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_ferry , test::IsBetween(defaults.use_ferry_.min,
defaults.use_ferry_.max));
  }

  // use_hills - used in the constructor to create grade penalties
  fDistributor.reset(make_real_distributor_from_range(kUseHillsRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_hills", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_hills , test::IsBetween( kUseHillsRange.min ,kUseHillsRange.max));
  }

  // use_primary - used in the constructor to create road factors.
  fDistributor.reset(make_real_distributor_from_range(kUsePrimaryRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_motorscootercost_from_json("use_primary", (*fDistributor)(generator)));
EXPECT_THAT(ctorTester->use_primary , test::IsBetween( kUsePrimaryRange.min ,kUsePrimaryRange.max));
  }
**/
}
} // namespace

#endif
