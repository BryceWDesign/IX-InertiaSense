// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Bryce Lovell
#pragma once
#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <vector>
namespace ix {
struct Vec3 {
  double x=0, y=0, z=0;
  Vec3 operator+(Vec3 b) const {return {x+b.x,y+b.y,z+b.z};}
  Vec3 operator-(Vec3 b) const {return {x-b.x,y-b.y,z-b.z};}
  Vec3 operator*(double k) const {return {x*k,y*k,z*k};}
  Vec3 operator/(double k) const {return *this*(1.0/k);}
};
inline Vec3 operator*(double k,Vec3 v){return v*k;}
double dot(Vec3 a,Vec3 b);
Vec3 cross(Vec3 a,Vec3 b);
double norm(Vec3 v);
bool finite(Vec3 v);
Vec3 normalized(Vec3 v);

struct Quaternion {
  double w=1,x=0,y=0,z=0;
  Quaternion conjugate() const {return {w,-x,-y,-z};}
  Quaternion operator*(Quaternion b) const;
  Quaternion unit() const;
  Vec3 rotate(Vec3 v) const; // body to world
};

// IMU specific_force is in body frame, approximately -gravity when stationary.
// Under linear acceleration, accelerometer tilt is invalid and is rejected.
class VestibularFilter {
 public:
  explicit VestibularFilter(double gain=0.045):gain_(gain){}
  bool update(Vec3 angular_rate_body_rad_s,Vec3 specific_force_body_m_s2,double dt_s);
  Quaternion orientation()const{return q_;}
  void reset(Quaternion q = {});
  unsigned rejected_acceleration()const{return rejected_;}
 private:
  Quaternion q_{}; double gain_; unsigned rejected_=0;
};

// Static or quasi-static calibrated wrench with gravity expressed in SENSOR frame.
// force in N; torque in Nm; gravity_m_s2 includes orientation. Wrench must be
// TARED for the tool and sensor, and represent payload weight, not contact forces.
struct WrenchObservation {
  Vec3 gravity,force,torque;
  double timestamp_s=0;
  bool static_equilibrium=true;
  bool healthy=true;
  double quality=1.0;
};
struct PayloadEstimate {
  bool observable=false;
  double mass_kg=0;
  Vec3 com_sensor_m{};
  double max_information_condition=std::numeric_limits<double>::infinity();
  double weighted_rms=0;
  double min_information_eigenvalue=0;
  std::size_t sample_count=0;
  std::size_t rejected=0;
  std::string status="no_data";
};

// Weighted robust (Huber) 4-parameter inertial estimator: theta=[mass,mass*r].
// Not a general dynamic rigid body identification algorithm.
class PayloadEstimator {
 public:
  explicit PayloadEstimator(std::size_t window=128,double max_extent_m=1.0);
  bool add(const WrenchObservation& sample);
  PayloadEstimate estimate()const;
  void clear();
  std::size_t count()const{return samples_.size();}
  std::size_t rejected()const{return rejected_;}
  // Given candidate body-frame gravity vectors, suggest a new orientation that
  // maximizes the smallest information eigenvalue. This is an E-optimal proxy.
  int suggest_probe(const std::vector<Vec3>& gravity_candidates)const;
 private:
  std::deque<WrenchObservation> samples_;
  std::size_t window_; double extent_; std::size_t rejected_=0;
  double last_time_=-std::numeric_limits<double>::infinity();
};

// Detect persistent mismatch with a previously identified payload. A change
// invalidates the estimate and starts a new identification episode.
// Threshold is a weighted force-equivalent wrench residual in newtons.
class AdaptivePayloadTracker {
 public:
  explicit AdaptivePayloadTracker(double change_threshold_N=4.0,unsigned required_hits=5);
  void observe(const WrenchObservation& sample);
  PayloadEstimate estimate() const {return estimator_.estimate();}
  std::size_t changes() const {return changes_;}
  std::size_t dropped_inputs() const {return dropped_;}
  unsigned mismatch_run() const {return run_;}
 private:
  PayloadEstimator estimator_{256,1};
  double threshold_;
  unsigned required_;
  unsigned run_=0;
  std::size_t changes_=0;
  std::size_t dropped_=0;
  double last_seen_=-std::numeric_limits<double>::infinity();
  std::deque<WrenchObservation> suspected_;
};

struct BodySegment {double mass_kg; Vec3 com_world_m;};
Vec3 body_com(const std::vector<BodySegment>& segments);
Vec3 loaded_com(const std::vector<BodySegment>& segments, double payload_mass_kg, Vec3 payload_com_world_m);

// Counterclockwise support polygon; signed distance to nearest edge, negative
// if outside. Assumes a flat horizontal plane and single support region.
struct Vec2 {double x=0,y=0;};
double signed_support_margin(const std::vector<Vec2>& polygon,Vec2 point);
// LIPM capture point approximation for sagittal/lateral plane, not a full
// nonlinear stepping or contact planner.
Vec2 capture_point(Vec2 com_ground_xy,Vec2 com_velocity_xy,double com_height_m,double gravity=9.80665);
}
