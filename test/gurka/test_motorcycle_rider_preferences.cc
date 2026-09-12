#include "gurka.h"
#include "tyr/actor.h"

#include <gtest/gtest.h>

#include <optional>

using namespace valhalla;

namespace {

Api RouteRaw(const gurka::map& map,
             const std::string& origin,
             const std::string& destination,
             const std::optional<std::string>& costing_options_json = std::nullopt) {
  const auto& origin_ll = map.nodes.at(origin);
  const auto& destination_ll = map.nodes.at(destination);
  std::string request = R"({"costing":"motorcycle","locations":[{"lon":)" +
                        std::to_string(origin_ll.lng()) + R"(,"lat":)" +
                        std::to_string(origin_ll.lat()) + R"(},{"lon":)" +
                        std::to_string(destination_ll.lng()) + R"(,"lat":)" +
                        std::to_string(destination_ll.lat()) + "}]";
  if (costing_options_json) {
    request += R"(,"costing_options":{"motorcycle":{)" + *costing_options_json + "}}";
  }
  request += "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  tyr::actor_t actor(map.config, *reader, true);
  Api result;
  actor.route(request, {}, &result);
  return result;
}

const TripLeg& Leg(const Api& result) {
  return result.trip().routes(0).legs(0);
}

TripLeg_Cost ElapsedCost(const Api& result) {
  const auto& leg = Leg(result);
  return leg.node(leg.node_size() - 1).cost().elapsed_cost();
}

class MotorcycleRiderPreferences : public ::testing::Test {
protected:
  static gurka::map class_choice_map;
  static gurka::map signal_choice_map;
  static gurka::map signal_eta_map;
  static gurka::map stop_eta_map;
  static gurka::map plain_eta_map;
  static gurka::map low_class_map;
  static gurka::map plain_low_class_map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100.0;

    const std::string class_choice_layout = R"(
      A---B
      |   |
      C---D
    )";
    const gurka::ways class_choice_ways = {
        {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"BD", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"AC", {{"highway", "residential"}, {"oneway", "yes"}}},
        {"CD", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    class_choice_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(class_choice_layout, grid_size_meters),
                          class_choice_ways, {}, {}, "test/data/gurka_motorcycle_class_choice");

    const std::string signal_choice_layout = R"(
      A---B---C
      |       |
      G       H
      |       |
      D---E---F
    )";
    const gurka::ways signal_choice_ways = {
        {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"BC", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"AG", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"GD", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"DE", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"EF", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"FH", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"HC", {{"highway", "primary"}, {"oneway", "yes"}}},
    };
    const gurka::nodes signal_nodes = {{"B", {{"highway", "traffic_signals"}}}};
    signal_choice_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(signal_choice_layout, grid_size_meters),
                          signal_choice_ways, signal_nodes, {},
                          "test/data/gurka_motorcycle_signal_choice");

    const std::string single_path_layout = R"(
      A---B---C
    )";
    const gurka::ways single_path_ways = {
        {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"BC", {{"highway", "primary"}, {"oneway", "yes"}}},
    };
    signal_eta_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(single_path_layout, grid_size_meters),
                          single_path_ways, signal_nodes, {},
                          "test/data/gurka_motorcycle_signal_eta");

    const gurka::nodes stop_nodes = {{"B", {{"highway", "stop"}}}};
    stop_eta_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(single_path_layout, grid_size_meters),
                          single_path_ways, stop_nodes, {}, "test/data/gurka_motorcycle_stop_eta");

    plain_eta_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(single_path_layout, grid_size_meters),
                          single_path_ways, {}, {}, "test/data/gurka_motorcycle_plain_eta");

    const std::string low_class_layout = R"(
      X---A---C
          |   |
          B---D
    )";
    const gurka::ways low_class_ways = {
        {"XA", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"AC", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"DB", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"AB", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    low_class_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(low_class_layout, grid_size_meters),
                          low_class_ways, {}, {}, "test/data/gurka_motorcycle_low_class");

    const std::string plain_low_class_layout = R"(
      X---A---B
    )";
    const gurka::ways plain_low_class_ways = {
        {"XA", {{"highway", "primary"}, {"oneway", "yes"}}},
        {"AB", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    plain_low_class_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(plain_low_class_layout, grid_size_meters),
                          plain_low_class_ways, {}, {}, "test/data/gurka_motorcycle_plain_low_class");
  }
};

gurka::map MotorcycleRiderPreferences::class_choice_map = {};
gurka::map MotorcycleRiderPreferences::signal_choice_map = {};
gurka::map MotorcycleRiderPreferences::signal_eta_map = {};
gurka::map MotorcycleRiderPreferences::stop_eta_map = {};
gurka::map MotorcycleRiderPreferences::plain_eta_map = {};
gurka::map MotorcycleRiderPreferences::low_class_map = {};
gurka::map MotorcycleRiderPreferences::plain_low_class_map = {};

TEST_F(MotorcycleRiderPreferences, RoadClassPreferenceChangesChosenRoad) {
  // Without explicit road class preferences, the routing model naturally favors higher-class roads
  // (like primary) over residential roads for efficiency.
  const auto plain = RouteRaw(class_choice_map, "A", "D");
  gurka::assert::raw::expect_path(plain, {"AB", "BD"});

  // When the rider explicitly requests to avoid primary roads and strongly prefer residential roads,
  // the costing correctly overrides the default hierarchy and steers the route onto the residential
  // path.
  const auto prefer_residential =
      RouteRaw(class_choice_map, "A", "D", R"("use_primary": 0.0, "use_residential": 1.0)");
  gurka::assert::raw::expect_path(prefer_residential, {"AC", "CD"});
}

TEST_F(MotorcycleRiderPreferences, NoPreferenceMeansNoChange) {
  // A request that specifies no per-road-class preferences must behave identically to the baseline,
  // preserving backward compatibility for existing integrations.
  const auto plain = RouteRaw(class_choice_map, "A", "D");

  // Supplying only the legacy `use_highways` option must also leave the new per-road-class factors
  // inert, ensuring older requests route exactly as they did before the new options were introduced.
  const auto avoid_highways = RouteRaw(class_choice_map, "A", "D", R"("use_highways": 0.0)");
  const auto prefer_highways = RouteRaw(class_choice_map, "A", "D", R"("use_highways": 1.0)");

  gurka::assert::raw::expect_path(avoid_highways, {"AB", "BD"});
  gurka::assert::raw::expect_path(prefer_highways, {"AB", "BD"});

  EXPECT_NEAR(ElapsedCost(avoid_highways).cost(), ElapsedCost(plain).cost(), 0.01f);
}

TEST_F(MotorcycleRiderPreferences, TrafficSignalPenaltySteersAwayFromSignal) {
  // The path via node B is significantly shorter and thus favored by default, despite the traffic
  // signal.
  const auto plain = RouteRaw(signal_choice_map, "A", "C");
  gurka::assert::raw::expect_path(plain, {"AB", "BC"});

  // A large traffic_signal_penalty injects enough artificial cost at the signalized node
  // that a longer, uninterrupted detour becomes the preferred route.
  const auto penalized = RouteRaw(signal_choice_map, "A", "C", R"("traffic_signal_penalty": 300.0)");
  gurka::assert::raw::expect_path(penalized, {"AG", "GD", "DE", "EF", "FH", "HC"});
}

TEST_F(MotorcycleRiderPreferences, SignalWaitIsInEtaButPenaltyIsNot) {
  const auto plain_signal = RouteRaw(signal_eta_map, "A", "C", R"("traffic_signal_penalty": 0.0)");
  const auto penalized_signal =
      RouteRaw(signal_eta_map, "A", "C", R"("traffic_signal_penalty": 300.0)");

  // The penalty option exists solely to influence the path search. It must not inflate the reported
  // ETA, which must remain an accurate reflection of physical travel time regardless of routing
  // preferences.
  EXPECT_NEAR(ElapsedCost(plain_signal).seconds(), ElapsedCost(penalized_signal).seconds(), 0.01f);

  const auto plain_no_signal = RouteRaw(plain_eta_map, "A", "C");

  // Even with a zero penalty, the routing model knows a traffic signal forces a physical stop.
  // The time spent waiting at the light must be included in the route's ETA.
  float diff = ElapsedCost(plain_signal).seconds() - ElapsedCost(plain_no_signal).seconds();
  EXPECT_NEAR(diff, 15.0f, 2.0f);
}

TEST_F(MotorcycleRiderPreferences, StopSignWaitIsInEta) {
  const auto plain_stop = RouteRaw(stop_eta_map, "A", "C", R"("stop_sign_penalty": 0.0)");
  const auto penalized_stop = RouteRaw(stop_eta_map, "A", "C", R"("stop_sign_penalty": 300.0)");

  // Like the traffic signal penalty, a stop sign penalty is artificial routing weight and must
  // be completely excluded from the estimated time of arrival.
  EXPECT_NEAR(ElapsedCost(plain_stop).seconds(), ElapsedCost(penalized_stop).seconds(), 0.01f);

  const auto plain_no_stop = RouteRaw(plain_eta_map, "A", "C");

  // A physical stop sign requires the rider to come to a halt, spending a small amount of time.
  // This wait must be reflected in the final duration estimate to maintain accuracy.
  float diff = ElapsedCost(plain_stop).seconds() - ElapsedCost(plain_no_stop).seconds();
  EXPECT_NEAR(diff, 4.0f, 2.0f);
}

TEST_F(MotorcycleRiderPreferences, LowClassPenaltyDiscouragesResidentialAndDoesNotChangeEta) {
  // The residential link is the short way round, so a plain route turns onto it.
  const auto plain = RouteRaw(low_class_map, "X", "B");
  gurka::assert::raw::expect_path(plain, {"XA", "AB"});

  // The charge is paid once per entry onto a low class road, so it buys the rider out of a chain
  // of short back streets while still allowing one long one. Here it should be enough to send the
  // route the long way round on primary roads.
  const auto penalized = RouteRaw(low_class_map, "X", "B", R"("low_class_penalty": 300.0)");
  gurka::assert::raw::expect_path(penalized, {"XA", "AC", "CD", "DB"});

  // On a map with only one way through, the charge still applies at the turn onto the residential
  // road, and the reported time must not move: it steers the search, it is not time spent.
  const auto plain_single = RouteRaw(plain_low_class_map, "X", "B", R"("low_class_penalty": 0.0)");
  const auto penalized_single =
      RouteRaw(plain_low_class_map, "X", "B", R"("low_class_penalty": 300.0)");
  gurka::assert::raw::expect_path(penalized_single, {"XA", "AB"});
  EXPECT_NEAR(ElapsedCost(plain_single).seconds(), ElapsedCost(penalized_single).seconds(), 0.01f);
  EXPECT_GT(ElapsedCost(penalized_single).cost(), ElapsedCost(plain_single).cost());
}

} // namespace
