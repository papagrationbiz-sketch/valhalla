#include "gurka.h"
#include "tyr/actor.h"

#include <gtest/gtest.h>

#include <optional>

using namespace valhalla;

namespace {

constexpr float kPenalty = 60.0f;

Api RouteRaw(const gurka::map& map,
             const std::string& origin,
             const std::string& destination,
             const std::optional<bool> enabled,
             const std::optional<int> date_time_type = std::nullopt) {
  const auto& origin_ll = map.nodes.at(origin);
  const auto& destination_ll = map.nodes.at(destination);
  std::string request = R"({"costing":"motor_scooter","locations":[{"lon":)" +
                        std::to_string(origin_ll.lng()) + R"(,"lat":)" +
                        std::to_string(origin_ll.lat()) + R"(},{"lon":)" +
                        std::to_string(destination_ll.lng()) + R"(,"lat":)" +
                        std::to_string(destination_ll.lat()) + "}]";
  if (enabled) {
    request += R"(,"costing_options":{"motor_scooter":{"avoid_multi_lane_right_turns":)";
    request += *enabled ? "true" : "false";
    request += "}}";
  }
  if (date_time_type) {
    request += R"(,"date_time":{"type":)" + std::to_string(*date_time_type) +
               R"(,"value":"2020-08-01T11:12"})";
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

float TransitionCost(const Api& result) {
  float cost = 0.0f;
  for (const auto& node : Leg(result).node()) {
    cost += node.cost().transition_cost().cost();
  }
  return cost;
}

float TransitionSeconds(const Api& result) {
  float seconds = 0.0f;
  for (const auto& node : Leg(result).node()) {
    seconds += node.cost().transition_cost().seconds();
  }
  return seconds;
}

void ExpectCostDelta(const Api& disabled, const Api& enabled, const float expected_cost_delta) {
  EXPECT_NEAR(TransitionCost(enabled) - TransitionCost(disabled), expected_cost_delta, 0.01f);
  EXPECT_NEAR(TransitionSeconds(enabled), TransitionSeconds(disabled), 0.01f);
  EXPECT_NEAR(ElapsedCost(enabled).seconds(), ElapsedCost(disabled).seconds(), 0.01f);
}

class MotorScooterMultiLaneRightTurns : public ::testing::Test {
protected:
  static gurka::map choice_map;
  static gurka::map turn_map;
  static gurka::map internal_map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100.0;

    const std::string choice_layout = R"(
      A---B
      |   |
      E   C
      |   |
      F---D
    )";
    const gurka::ways choice_ways = {
        {"AB", {{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "2"}}},
        {"BC", {{"highway", "residential"}, {"oneway", "yes"}}},
        {"AEF", {{"highway", "residential"}, {"oneway", "yes"}}},
        {"FDC", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    choice_map = gurka::buildtiles(gurka::detail::map_to_coordinates(choice_layout, grid_size_meters),
                                   choice_ways, {}, {},
                                   "test/data/gurka_motor_scooter_multi_lane_right_turn_choice");

    const std::string turn_layout = R"(
          L
          |
      A---B---S
          |
          R
    )";
    const gurka::ways turn_ways = {
        {"AB",
         {{"highway", "residential"},
          {"oneway", "yes"},
          {"lanes", "1"},
          {"turn:lanes", "left|through|right"}}},
        {"BL", {{"highway", "residential"}, {"oneway", "yes"}}},
        {"BS", {{"highway", "residential"}, {"oneway", "yes"}}},
        {"BR", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    turn_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(turn_layout, grid_size_meters), turn_ways,
                          {}, {}, "test/data/gurka_motor_scooter_multi_lane_right_turn_types");

    const std::string internal_layout = R"(
      A---B
          |
          I
          |
          C
    )";
    const gurka::ways internal_ways = {
        {"AB", {{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "2"}}},
        {"BI", {{"highway", "residential"}, {"oneway", "yes"}, {"internal_intersection", "true"}}},
        {"IC", {{"highway", "residential"}, {"oneway", "yes"}}},
    };
    internal_map =
        gurka::buildtiles(gurka::detail::map_to_coordinates(internal_layout, grid_size_meters),
                          internal_ways, {}, {},
                          "test/data/gurka_motor_scooter_multi_lane_right_turn_internal",
                          {{"mjolnir.data_processing.infer_internal_intersections", "false"}});
  }
};

gurka::map MotorScooterMultiLaneRightTurns::choice_map = {};
gurka::map MotorScooterMultiLaneRightTurns::turn_map = {};
gurka::map MotorScooterMultiLaneRightTurns::internal_map = {};

TEST_F(MotorScooterMultiLaneRightTurns, RawJsonDefaultAndFalseAreUnchanged) {
  const auto omitted = RouteRaw(choice_map, "A", "C", std::nullopt);
  const auto disabled = RouteRaw(choice_map, "A", "C", false);

  gurka::assert::raw::expect_path(omitted, {"AB", "BC"});
  gurka::assert::raw::expect_path(disabled, {"AB", "BC"});
  EXPECT_NEAR(ElapsedCost(omitted).cost(), ElapsedCost(disabled).cost(), 0.01f);
  EXPECT_NEAR(ElapsedCost(omitted).seconds(), ElapsedCost(disabled).seconds(), 0.01f);
}

TEST_F(MotorScooterMultiLaneRightTurns, RawJsonAvoidsMultiLaneRightTurn) {
  const auto enabled = RouteRaw(choice_map, "A", "C", true);
  gurka::assert::raw::expect_path(enabled, {"AEF", "AEF", "FDC", "FDC"});
}

TEST_F(MotorScooterMultiLaneRightTurns, PenalizesOnlyRightTurns) {
  const auto right_disabled = RouteRaw(turn_map, "A", "R", false);
  const auto right_enabled = RouteRaw(turn_map, "A", "R", true);
  ExpectCostDelta(right_disabled, right_enabled, kPenalty);

  const auto left_disabled = RouteRaw(turn_map, "A", "L", false);
  const auto left_enabled = RouteRaw(turn_map, "A", "L", true);
  ExpectCostDelta(left_disabled, left_enabled, 0.0f);

  const auto straight_disabled = RouteRaw(turn_map, "A", "S", false);
  const auto straight_enabled = RouteRaw(turn_map, "A", "S", true);
  ExpectCostDelta(straight_disabled, straight_enabled, 0.0f);
}

TEST_F(MotorScooterMultiLaneRightTurns, InternalIntersectionAddsPenaltyExactlyOnce) {
  const auto disabled = RouteRaw(internal_map, "A", "C", false);
  const auto enabled = RouteRaw(internal_map, "A", "C", true);

  gurka::assert::raw::expect_path(enabled, {"AB", "BI", "IC"});
  ExpectCostDelta(disabled, enabled, kPenalty);
}

TEST_F(MotorScooterMultiLaneRightTurns, ForwardAndReverseSearchAreSymmetric) {
  const auto forward = RouteRaw(internal_map, "A", "C", true, 1);
  const auto reverse = RouteRaw(internal_map, "A", "C", true, 2);

  ASSERT_EQ(Leg(forward).node_size(), Leg(reverse).node_size());
  for (int i = 0; i < Leg(forward).node_size(); ++i) {
    EXPECT_NEAR(Leg(forward).node(i).cost().transition_cost().cost(),
                Leg(reverse).node(i).cost().transition_cost().cost(), 0.01f);
    EXPECT_NEAR(Leg(forward).node(i).cost().transition_cost().seconds(),
                Leg(reverse).node(i).cost().transition_cost().seconds(), 0.01f);
  }
}

} // namespace
