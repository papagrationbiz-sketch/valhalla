#ifndef VALHALLA_SIF_RIDERPREFERENCES_H_
#define VALHALLA_SIF_RIDERPREFERENCES_H_

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

#include <algorithm>
#include <cstddef>

namespace valhalla {
namespace sif {

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

// Signals and stop signs are ubiquitous, so both penalties default to none. A route only
// avoids them when the request explicitly asks for it. The low class charge is paid per entry
// onto a residential or service road, the same option the truck costing already exposes.
constexpr float kDefaultTrafficSignalPenalty = 0.0f; // Seconds
constexpr float kDefaultStopSignPenalty = 0.0f;      // Seconds
constexpr float kDefaultLowClassPenalty = 0.0f;      // Seconds

constexpr ranged_default_t<float> kTrafficSignalPenaltyRange{0.0f, kDefaultTrafficSignalPenalty,
                                                             kMaxPenalty};
constexpr ranged_default_t<float> kStopSignPenaltyRange{0.0f, kDefaultStopSignPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kLowClassPenaltyRange{0.0f, kDefaultLowClassPenalty, kMaxPenalty};

// Floor applied to a road class weight when a request asks to avoid that class.
// Residential is weighted 0.0 and unclassified 0.05, so without this the avoid
// direction would be a no-op for exactly the classes riders most want to steer
// away from on a moped.
constexpr float kMinAvoidableRoadClassFactor = 0.1f;

// The eight Valhalla road classes, in RoadClass order: motorway, trunk, primary, secondary,
// tertiary, unclassified, residential, service/other.
constexpr size_t kRoadClassCount = 8;

// How to turn the use_* preferences into per class weights.
//
// kInheritUsePrimary is what motor_scooter shipped: one scale derived from the preference
// multiplies the class weight, an unnamed class follows use_primary, and the numbers have to keep
// reproducing themselves for requests that name only use_primary.
//
// kAvoidOnly suits a costing that travels at road speed. There, the time saved on a faster class
// swamps a weight of that size - a scooter capped at 45 km/h covers a primary and a residential
// road at nearly the same speed, a motorcycle does not - so avoiding a class has to be worth more
// than the class is faster. It also charges nothing at all until a request asks to avoid
// something, which keeps a costing that never had these options behaving exactly as before.
enum class RoadClassPreferenceMode {
  kInheritUsePrimary, // an unnamed class follows use_primary, weights scale as motor_scooter's do
  kAvoidOnly,         // only a preference below 0.5 costs anything, and it costs road-speed money
};

// How much avoiding a road class is worth under kAvoidOnly, as a multiple of the class weight.
// Matched to the bias motorcycle already applies for use_highways, which is the scale a penalty
// has to reach to outweigh a faster road.
constexpr float kAvoidRoadClassStrength = 8.0f;

// Weights each road class from the use_* preferences, so a rider can ask for the back streets
// or for the arterials instead of accepting one scalar for every class at once.
//
// Returns true when the request named at least one road class preference. A caller whose
// costing had no per class preference before can use that to leave the whole term switched
// off and stay bit-for-bit identical to its old behaviour.
inline bool ComputeRoadClassFactors(const Costing::Options& options,
                                    const float (&base_weights)[kRoadClassCount],
                                    RoadClassPreferenceMode mode,
                                    float (&factors)[kRoadClassCount]) {
  const float use_primary = options.use_primary();
  const bool inherits = mode == RoadClassPreferenceMode::kInheritUsePrimary;
  const bool named_primary = options.has_use_primary();

  struct ClassPreference {
    float use_value;
    bool named;
  };

  // Motorway, trunk and primary all follow use_primary.
  //
  // Under kInheritUsePrimary they count as unnamed however the request arrived. That is what
  // keeps a use_primary-only request reproducing the numbers from before these options existed:
  // the avoid floor below never reaches it. Under kAvoidOnly there is no such history to
  // preserve, so naming use_primary means what it says.
  const ClassPreference primary_preference{inherits || named_primary ? use_primary : 0.5f,
                                           inherits ? false : named_primary};

  // A class the request left out either follows use_primary or sits at the neutral 0.5,
  // depending on what the calling costing used to do with use_primary.
  const auto unnamed_value = inherits ? use_primary : 0.5f;
  const auto preference = [&](bool named, float value) -> ClassPreference {
    return {named ? value : unnamed_value, named};
  };

  const ClassPreference preference_by_class[kRoadClassCount] = {
      primary_preference,
      primary_preference,
      primary_preference,
      preference(options.has_use_secondary(), options.use_secondary()),
      preference(options.has_use_tertiary(), options.use_tertiary()),
      preference(options.has_use_unclassified(), options.use_unclassified()),
      preference(options.has_use_residential(), options.use_residential()),
      preference(options.has_use_service(), options.use_service()),
  };

  for (size_t i = 0; i < kRoadClassCount; ++i) {
    const float use_value = preference_by_class[i].use_value;

    // Residential is weighted 0.0 and unclassified 0.05, so weighting alone can never express
    // avoiding them - zero stays zero however hard it is pushed. Raise a floor for those classes
    // as the preference moves below 0.5, ramping in from nothing at 0.5 to the full floor at 0 so
    // there is no step in cost as a caller sweeps the value.
    //
    // Under kInheritUsePrimary the floor applies only to a class the request named, so a class
    // that merely inherited use_primary keeps its original weight.
    const float avoidance = std::max(0.0f, 0.5f - use_value) * 2.0f;
    float weight = base_weights[i];
    if (preference_by_class[i].named && avoidance > 0.0f) {
      weight = std::max(weight, kMinAvoidableRoadClassFactor * avoidance);
    }

    if (inherits) {
      // Above 0.5 the weight difference between road classes shrinks; below 0.5 it grows.
      const float scale = (use_value >= 0.5f) ? 1.5f - use_value : 3.0f - use_value * 5.0f;
      factors[i] = scale * weight;
    } else {
      // Nothing is charged for a class the request is content with, so a request that asks to
      // avoid nothing pays nothing anywhere.
      factors[i] = kAvoidRoadClassStrength * weight * avoidance;
    }
  }

  return named_primary || options.has_use_secondary() || options.has_use_tertiary() ||
         options.has_use_unclassified() || options.has_use_residential() || options.has_use_service();
}

// Adds the time a rider actually spends at signals and stop signs to the estimate, and the
// requested penalties to the search cost.
inline void AddSignalAndStopSignCost(const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* ingress_edge,
                                     float traffic_signal_penalty,
                                     float stop_sign_penalty,
                                     Cost& c) {
  if (node->traffic_signal()) {
    // The wait is time the rider actually spends, so it belongs in the estimate. The
    // penalty on top of it only steers the search and leaves the estimate alone.
    // Only the estimate. Adding the wait to the search cost as well would steer every
    // route away from signals, including for riders who never asked to avoid them.
    c.secs += kTrafficSignalDelay;
    c.cost += traffic_signal_penalty;
  }
  if (ingress_edge && ingress_edge->stop_sign()) {
    c.secs += kStopSignDelay;
    c.cost += stop_sign_penalty;
  }
}

inline void AddLowClassCost(const baldr::DirectedEdge* edge, float low_class_penalty, Cost& c) {
  if (low_class_penalty > 0.0f && (edge->classification() == baldr::RoadClass::kResidential ||
                                   edge->classification() == baldr::RoadClass::kServiceOther)) {
    // This is a generalized search cost only; do not alter ETA.
    c.cost += low_class_penalty;
  }
}

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_RIDERPREFERENCES_H_
