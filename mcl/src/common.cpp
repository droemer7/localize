#include <algorithm>
#include <chrono>
#include <limits>
#include <math.h>

#include "mcl/common.h"

using namespace localize;

const unsigned int localize::SENSOR_TH_SAMPLE_COUNT = 8;
const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

// ========== Point ========== //
Point::Point(const double x,
             const double y
            ):
  x_(x),
  y_(y)
{}

// ========== Pose ========== //
Pose::Pose(const Point& point,
           const double th
          ):
  Point(point),
  th_(th)
{}

Pose::Pose(const double x,
           const double y,
           const double th
          ):
  Pose(Point(x, y),
       th
      )
{}

// ========== Particle ========== //
Particle::Particle(const Pose& pose,
                   const double weight,
                   const double weight_normed
                  ):
  Pose(pose),
  weight_(weight),
  weight_normed_(weight_normed),
  weights_(SENSOR_TH_SAMPLE_COUNT, 0.0)
{}

Particle::Particle(const double x,
                   const double y,
                   const double th,
                   const double weight,
                   const double weight_normed
                  ):
  Particle(Pose(x, y, th),
           weight,
           weight_normed
          )
{}

int Particle::compare(const Particle& lhs, const Particle& rhs) const
{
  int result;
  if (lhs.weight_normed_ < rhs.weight_normed_) {
    result = -1;
  }
  else if (lhs.weight_normed_ > rhs.weight_normed_) {
    result = 1;
  }
  else { // lhs.weight_normed_ == rhs.weight_normed_
    result = 0;
  }
  return result;
}

// ========== Ray ========== //
Ray::Ray(const float range,
         const float th
        ) :
  range_(range),
  th_(th)
{}

// ========== RaySample ========== //
RaySample::RaySample(const float range,
                     const float th,
                     const double weight_new_obj_sum,
                     const double weight_sum
                    ) :
  Ray(range, th),
  weight_new_obj_sum_(weight_new_obj_sum),
  weight_sum_(weight_sum)
{}

RaySample::RaySample(const Ray& ray) :
  RaySample(ray.range_, ray.th_)
{}

// ========== RayScan ========== //
RayScan::RayScan(const RayVector& rays,
                 float th_inc,
                 float t_inc,
                 float t_dur
                ) :
  rays_(rays),
  th_inc_(th_inc),
  t_inc_(t_inc),
  t_dur_(t_dur)
{}

RayScan::RayScan(size_t num_rays) :
  RayScan(RayVector(num_rays))
{}

// ========== Map ========== //
Map::Map(const unsigned int width,
         const unsigned int height,
         const double x_origin_world,
         const double y_origin_world,
         const double th_world,
         const double scale_world,
         const std::vector<int8_t>& occ_data
        ) :
  OMap(width,
       height,
       x_origin_world,
       y_origin_world,
       th_world,
       scale_world,
       occ_data
      )
{}

bool Map::occupied(const Point& point) const
{
  return isOccupiedNT(point.y_, point.x_);
}

unsigned int Map::width() const
{
  return OMap::width;
}

unsigned int Map::height() const
{
  return OMap::height;
}

double Map::xOriginWorld() const
{
  return OMap::x_origin_world;
}

double Map::yOriginWorld() const
{
  return OMap::y_origin_world;
}

double Map::thWorld() const
{
  return OMap::th_world;
}

double Map::sinThWorld() const
{
  return OMap::sin_th_world;
}

double Map::cosThWorld() const
{
  return OMap::cos_th_world;
}

double Map::scaleWorld() const
{
  return OMap::scale_world;
}

Point localize::worldToMap(const Map& map, const Point& point_world)
{
  Point point_map;

  // Translate and rescale
  point_map.x_ = (point_world.x_ - map.xOriginWorld()) / map.scaleWorld();
  point_map.y_ = (point_world.y_ - map.yOriginWorld()) / map.scaleWorld();

  // Rotate
  double temp_map_x = point_map.x_;
  point_map.x_ =   map.cosThWorld() * point_map.x_ + map.sinThWorld() * point_map.y_;
  point_map.y_ = - map.sinThWorld() * temp_map_x   + map.cosThWorld() * point_map.y_;

  return point_map;
}

Pose localize::worldToMap(const Map& map, const Pose& pose_world)
{
  return Pose(worldToMap(map, Point(pose_world.x_, pose_world.y_)), // Transform point
              wrapAngle(pose_world.th_ - map.thWorld())             // Transform angle
             );
}

Point localize::mapToWorld(const Map& map, const Point& point_map)
{
  Point point_world;

  // Rotate
  point_world.x_ = map.cosThWorld() * point_map.x_ - map.sinThWorld() * point_map.y_;
  point_world.y_ = map.sinThWorld() * point_map.x_ + map.cosThWorld() * point_map.y_;

  // Rescale and translate
  point_world.x_ = point_world.x_ * map.scaleWorld() + map.xOriginWorld();
  point_world.y_ = point_world.y_ * map.scaleWorld() + map.yOriginWorld();

  return point_world;
}

Pose localize::mapToWorld(const Map& map, const Pose& pose_map)
{
  return Pose(mapToWorld(map, Point(pose_map.x_, pose_map.y_)), // Transform point (x, y)
              wrapAngle(pose_map.th_ + map.thWorld())           // Transform angle
             );
}

// ========== PoseRandomSampler ========== //
PoseRandomSampler::PoseRandomSampler(const Map& map) :
  map_(map),
  x_dist_(0.0, map_.width()),
  y_dist_(0.0, map_.height()),
  th_dist_(std::nextafter(-L_PI, std::numeric_limits<double>::max()),
           std::nextafter(L_PI, std::numeric_limits<double>::max())
          )
{}

Pose PoseRandomSampler::operator()()
{
  Pose pose(0.0, 0.0, th_dist_(rng_.engine()));
  bool occupied = true;

  // Generate x & y in map frame until free space is found
  while (occupied) {
    pose.x_ = x_dist_(rng_.engine());
    pose.y_ = y_dist_(rng_.engine());
    occupied = map_.occupied(pose);
  }
  // Convert pose from map to world frame
  return mapToWorld(map_, pose);
}