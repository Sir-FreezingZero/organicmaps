#include "testing/testing.hpp"

#include "routing/routing_integration_tests/generated_map_test.hpp"
#include "routing/routing_integration_tests/routing_test_tools.hpp"

#include "routing/route.hpp"
#include "routing/routing_callbacks.hpp"
#include "routing/routing_options.hpp"

#include "geometry/mercator.hpp"
#include "geometry/region2d.hpp"

#include "base/math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace avoid_zones_route_test
{
using namespace routing;

std::shared_ptr<AvoidZones const> MakeCircleZone(m2::PointD const & center, double radiusM)
{
  auto const rect = mercator::RectByCenterXYAndSizeInMeters(center, radiusM);
  std::vector<m2::PointD> points;
  for (int i = 0; i < 32; ++i)
  {
    double const angle = 2.0 * math::pi * i / 32;
    points.emplace_back(center.x + rect.SizeX() / 2 * std::cos(angle), center.y + rect.SizeY() / 2 * std::sin(angle));
  }
  return std::make_shared<AvoidZones const>(AvoidZones{m2::RegionD(points)});
}

double GetMinDistanceMeters(Route const & route, m2::PointD const & pt)
{
  double minDistance = std::numeric_limits<double>::max();
  auto const & poly = route.GetPoly();
  for (size_t i = 0; i + 1 < poly.GetSize(); ++i)
  {
    // Sample the segments: route vertices alone may be far from a zone the route crosses.
    auto const a = poly.GetPoint(i);
    auto const b = poly.GetPoint(i + 1);
    for (int k = 0; k <= 50; ++k)
      minDistance = std::min(minDistance, mercator::DistanceOnEarth(a + (b - a) * (k / 50.0), pt));
  }
  return minDistance;
}

// See data/test_data/osm/nsk_magadanskaya.osm: way 51993176 "Магаданская улица" connects start and finish
// directly, parallel service roads make detours.
UNIT_TEST(AvoidZones_GeneratedMapDetour)
{
  integration::GeneratedMapTest map("./data/test_data/osm/nsk_magadanskaya.osm", "Russia_Novosibirsk Oblast",
                                    VehicleType::Car);
  auto & router = map.GetComponents().GetRouter();
  auto const start = mercator::FromLatLon(55.074347, 82.939134);
  auto const finish = mercator::FromLatLon(55.076779, 82.935965);
  auto const calculate = [&] { return integration::CalculateRoute(map.GetComponents(), start, {0.0, 0.0}, finish); };

  auto const direct = calculate();
  TEST_EQUAL(direct.second, RouterResultCode::NoError, ());
  TEST(map.GetUsedOsmWays(*direct.first).contains(51993176), ());

  // The zone is on Магаданская, and the nearest service road crosses it between its nodes.
  auto const zoneCenter = mercator::FromLatLon(55.0755368, 82.9375686);
  for (double const radiusM : {15.0, 60.0})
  {
    router.SetAvoidZones(MakeCircleZone(zoneCenter, radiusM));
    auto const detour = calculate();
    TEST_EQUAL(detour.second, RouterResultCode::NoError, (radiusM));
    TEST_GREATER(GetMinDistanceMeters(*detour.first, zoneCenter), radiusM, ());
    TEST_GREATER(detour.first->GetTotalDistanceMeters(), direct.first->GetTotalDistanceMeters(), (radiusM));
  }

  // A zone containing the start is ignored.
  router.SetAvoidZones(MakeCircleZone(start, 50.0));
  auto const startInZone = calculate();
  TEST_EQUAL(startInZone.second, RouterResultCode::NoError, ());
  TEST_ALMOST_EQUAL_ABS(startInZone.first->GetTotalDistanceMeters(), direct.first->GetTotalDistanceMeters(), 0.1, ());

  router.SetAvoidZones(std::make_shared<AvoidZones const>());
  auto const cleared = calculate();
  TEST_EQUAL(cleared.second, RouterResultCode::NoError, ());
  TEST_ALMOST_EQUAL_ABS(cleared.first->GetTotalDistanceMeters(), direct.first->GetTotalDistanceMeters(), 0.1, ());
}
}  // namespace avoid_zones_route_test
