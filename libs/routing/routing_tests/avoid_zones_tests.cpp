#include "testing/testing.hpp"

#include "routing/routing_tests/index_graph_tools.hpp"

#include "routing/base/astar_algorithm.hpp"
#include "routing/index_graph.hpp"
#include "routing/index_graph_starter.hpp"
#include "routing/index_graph_starter_joints.hpp"
#include "routing/joint_segment.hpp"
#include "routing/routing_helpers.hpp"
#include "routing/routing_options.hpp"

#include "traffic/traffic_cache.hpp"

#include "indexer/classificator_loader.hpp"

#include "geometry/point2d.hpp"
#include "geometry/region2d.hpp"

#include <memory>
#include <vector>

namespace avoid_zones_tests
{
using namespace routing;
using namespace routing_test;
using namespace std;

using SegmentsAlgorithm = AStarAlgorithm<Segment, SegmentEdge, RouteWeight>;
using JointsAlgorithm = AStarAlgorithm<JointSegment, JointEdge, RouteWeight>;

//            (1, 1)
//          *---R1---*
//        /            \
// S --R2-*-----R0-------*--R3-- F
//      (0, 0) (1, 0)  (2, 0)
//
// R0 is the shortest way, R1 is the detour.
unique_ptr<SingleVehicleWorldGraph> BuildDetourGraph()
{
  classificator::Load();

  auto loader = make_unique<TestGeometryLoader>();
  loader->AddRoad(0 /* featureId */, false /* oneWay */, 1.0 /* speed */,
                  RoadGeometry::Points({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  loader->AddRoad(1, false, 1.0, RoadGeometry::Points({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}}));
  loader->AddRoad(2, false, 1.0, RoadGeometry::Points({{-1.0, 0.0}, {0.0, 0.0}}));
  loader->AddRoad(3, false, 1.0, RoadGeometry::Points({{2.0, 0.0}, {3.0, 0.0}}));

  vector<Joint> const joints = {
      MakeJoint({{0 /* featureId */, 0 /* pointId */}, {1, 0}, {2, 1}}), /* (0, 0) */
      MakeJoint({{0, 2}, {1, 2}, {3, 0}}),                               /* (2, 0) */
  };

  traffic::TrafficCache const trafficCache;
  return BuildWorldGraph(std::move(loader), CreateEstimatorForCar(trafficCache), joints);
}

m2::RegionD MakeSquare(m2::PointD const & center, double halfSide)
{
  return m2::RegionD(vector<m2::PointD>{{center.x - halfSide, center.y - halfSide},
                                        {center.x + halfSide, center.y - halfSide},
                                        {center.x + halfSide, center.y + halfSide},
                                        {center.x - halfSide, center.y + halfSide}});
}

unique_ptr<IndexGraphStarter> MakeDetourStarter(SingleVehicleWorldGraph & graph, AvoidZones && zones)
{
  graph.GetIndexGraphForTests(kTestNumMwmId).SetAvoidZones(make_shared<AvoidZones const>(std::move(zones)));
  auto const start = MakeFakeEnding(2 /* featureId */, 0 /* segmentIdx */, m2::PointD(-1.0, 0.0), graph);
  auto const finish = MakeFakeEnding(3, 0, m2::PointD(3.0, 0.0), graph);
  return MakeStarter(start, finish, graph);
}

// Returns the real features of the route found by joints, as the car routing does.
// Start and finish roads are covered by fake segments.
JointsAlgorithm::Result CalculateJointsRoute(IndexGraphStarter & starter, vector<uint32_t> & features)
{
  starter.GetGraph().SetMode(WorldGraphMode::JointSingleMwm);
  IndexGraphStarterJoints<IndexGraphStarter> jointsStarter(starter, starter.GetStartSegment(),
                                                           starter.GetFinishSegment());
  JointsAlgorithm::ParamsForTests<AStarLengthChecker> params(
      jointsStarter, jointsStarter.GetStartJoint(), jointsStarter.GetFinishJoint(), AStarLengthChecker(starter));
  RoutingResult<JointSegment, RouteWeight> result;
  auto const code = JointsAlgorithm().FindPathBidirectional(params, result);

  features.clear();
  for (auto const & joint : result.m_path)
  {
    for (auto const & segment : jointsStarter.ReconstructJoint(joint))
      if (segment.IsRealSegment() && (features.empty() || features.back() != segment.GetFeatureId()))
        features.push_back(segment.GetFeatureId());
  }
  return code;
}

UNIT_TEST(AvoidZones_NoZones)
{
  auto graph = BuildDetourGraph();
  auto starter = MakeDetourStarter(*graph, {});
  TestRouteGeometry(*starter, SegmentsAlgorithm::Result::OK,
                    {{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
}

UNIT_TEST(AvoidZones_Detour)
{
  auto graph = BuildDetourGraph();
  auto starter = MakeDetourStarter(*graph, {MakeSquare({1.0, 0.0}, 0.5)});
  TestRouteGeometry(*starter, SegmentsAlgorithm::Result::OK,
                    {{-1.0, 0.0}, {0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 0.0}});
}

UNIT_TEST(AvoidZones_ZoneAwayFromRoute)
{
  auto graph = BuildDetourGraph();
  auto starter = MakeDetourStarter(*graph, {MakeSquare({1.0, -1.0}, 0.5)});
  TestRouteGeometry(*starter, SegmentsAlgorithm::Result::OK,
                    {{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
}

UNIT_TEST(AvoidZones_AllWaysBlocked)
{
  auto graph = BuildDetourGraph();
  auto starter = MakeDetourStarter(*graph, {MakeSquare({1.0, 0.0}, 0.5), MakeSquare({1.0, 1.0}, 0.5)});
  TestRouteGeometry(*starter, SegmentsAlgorithm::Result::NoPath, {});
}

UNIT_TEST(AvoidZones_Joints_Detour)
{
  auto graph = BuildDetourGraph();
  vector<uint32_t> features;

  auto starter = MakeDetourStarter(*graph, {});
  TEST_EQUAL(CalculateJointsRoute(*starter, features), JointsAlgorithm::Result::OK, ());
  TEST_EQUAL(features, vector<uint32_t>({0}), ());

  // Points between joints are checked too.
  starter = MakeDetourStarter(*graph, {MakeSquare({1.0, 0.0}, 0.5)});
  TEST_EQUAL(CalculateJointsRoute(*starter, features), JointsAlgorithm::Result::OK, ());
  TEST_EQUAL(features, vector<uint32_t>({1}), ());

  starter = MakeDetourStarter(*graph, {MakeSquare({1.0, 0.0}, 0.5), MakeSquare({1.0, 1.0}, 0.5)});
  TEST_EQUAL(CalculateJointsRoute(*starter, features), JointsAlgorithm::Result::NoPath, ());
}
}  // namespace avoid_zones_tests
