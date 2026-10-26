// SPDX-License-Identifier: Apache-2.0
#include "ix_inertiasense/core.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <vector>
using namespace ix;
void check(bool b,const char*msg){if(!b)throw std::runtime_error(msg);}
void close(double a,double b,double tol,const char*msg){check(std::abs(a-b)<tol,msg);}
WrenchObservation obs(Vec3 g,double m,Vec3 r,int index,double noise=0){
 Vec3 f=m*g,t=cross(r*m,g);double n=noise*std::sin(index*4.51);
 return {g,f+Vec3{n,-n/2,n/4},t+Vec3{n/10,0,-n/20},index*0.01,true,true,1.0};
}
int main(){try{
 // 1. Algebraic physics and quaternion rotation
 close(norm(cross({1,0,0},{0,1,0})),1,1e-12,"cross product");
 Quaternion q{std::sqrt(.5),0,0,std::sqrt(.5)};
 auto v=q.rotate({1,0,0});close(v.y,1,1e-12,"quaternion rotation");
 // 2. Vestibular at rest, and reject linear acceleration as a tilt reference.
 VestibularFilter imu;for(int i=0;i<500;++i)check(imu.update({0.0,0,0},{0,0,9.80665},.01),"imu update");
 close(imu.orientation().w,1,1e-9,"rest orientation");
 check(imu.update({0,0,0},{15,0,9.8},.01),"accelerated imu");
 check(imu.rejected_acceleration()==1,"accelerated rejected");
 check(!imu.update({0,0,0},{0,0,9.8},-1),"negative dt");
 // Quasi-static pitch/roll should converge toward the gravity direction.
 VestibularFilter tilt;
 tilt.reset({std::cos(.25),std::sin(.25),0,0});
 for(int i=0;i<200;++i)tilt.update({0,0,0},{0,0,9.80665},.01);
 check(std::abs(tilt.orientation().x)<.01,"tilt filter converges");
 // 3. Known center of mass and changing link pose
 auto a=body_com({{2,{0,0,0}},{2,{1,0,0}}});close(a.x,0.5,1e-12,"body com");
 auto b=loaded_com({{2,{0,0,0}},{2,{1,0,0}}},2,{2,0,0});close(b.x,1.,1e-12,"loaded com");
 // 4. Support polygon and dynamic capture point
 std::vector<Vec2> box{{-1,-1},{1,-1},{1,1},{-1,1}};
 close(signed_support_margin(box,{0,0}),1,1e-12,"inside margin");
 check(signed_support_margin(box,{2,0})<0,"outside margin");
 bool clockwise_rejected=false;
 try{signed_support_margin({{-1,-1},{-1,1},{1,1},{1,-1}},{0,0});}
 catch(const std::invalid_argument&){clockwise_rejected=true;}
 check(clockwise_rejected,"wrong polygon winding rejected");
 auto cp=capture_point({0,0},{1,0},1);close(cp.x,std::sqrt(1./9.80665),1e-12,"capture point");
 // 5. Payload identification requires varied gravity directions
 PayloadEstimator p(256,1);
 for(int i=0;i<25;++i)check(p.add(obs({0,0,-9.80665},3,{.1,-.05,.16},i)),"add fixed gravity");
 check(!p.estimate().observable,"false certainty for a single gravity direction");
 auto pick=p.suggest_probe({{0,0,-9.80665},{-6,0,-7.74},{0,6,-7.74}});
 check(pick!=0&&pick>=0,"informative probe");
 // 6. Multi-orientation 3D recovery
 PayloadEstimator fit(256,1);
 for(int i=0;i<120;++i){double phi=i*2.39996322972865;
  Vec3 d=normalized({std::sin(phi)*.45,std::cos(phi)*.45,-1});
  check(fit.add(obs(d*9.80665,3,{.1,-.05,.16},i,0.08)),"add");}
 auto e=fit.estimate();check(e.observable,"observable after tilts");
 close(e.mass_kg,3,.01,"mass recovered");
 close(e.com_sensor_m.x,.1,.005,"payload x");
 close(e.com_sensor_m.y,-.05,.005,"payload y");
 close(e.com_sensor_m.z,.16,.005,"payload z");
 // 7. Reject non-equilibrium, stale, invalid, physically implausible data
 check(!fit.add({{0,0,-9.8},{0,0,0},{0,0,0},2,false,true,1}),"reject dynamics");
 check(!fit.add({{0,0,-9.8},{0,0,0},{0,0,0},.001,true,true,1}),"reject replay timestamp");
 check(!fit.add({{0,0,-9.8},{0,0,0},{0,0,0},2,true,false,1}),"reject failed health");
 check(fit.rejected()==3,"rejection account");
 PayloadEstimator impossible(128,.03);
 for(int i=0;i<80;++i){double phi=i*2.39996;Vec3 d=normalized({std::sin(phi)*.5,std::cos(phi)*.5,-1});impossible.add(obs(d*9.80665,3,{.1,-.05,.16},i));}
 check(impossible.estimate().status=="outside_payload_bounds","physical geometry bound");
 // 8. Robust outlier wrenches should not drastically bias otherwise healthy fit
 PayloadEstimator robust(256,1);
 for(int i=0;i<160;++i){double phi=i*2.4;Vec3 d=normalized({std::sin(phi)*.55,std::cos(phi)*.55,-1});auto s=obs(d*9.80665,2.3,{.16,.03,-.11},i,.12);
  if(i%13==0){s.force.x+=14;s.torque.z+=5;}
  robust.add(s);
 }
 auto r=robust.estimate();check(r.observable,"robust remains observable");
 close(r.mass_kg,2.3,.18,"outlier robust mass");
 check(norm(r.com_sensor_m-Vec3{.16,.03,-.11})<.065,"outlier robust center");
 // 9. Real payload change invalidates previously converged estimate.
 AdaptivePayloadTracker tracker(3.0,4);
 for(int i=0;i<110;++i){double phi=i*2.4;Vec3 d=normalized({std::sin(phi)*.6,std::cos(phi)*.5,-1});
 tracker.observe(obs(d*9.80665,2.0,{.05,0,.1},i));}
 check(tracker.estimate().observable,"tracker learns initial load");
 for(int i=110;i<250;++i){double phi=i*2.4;Vec3 d=normalized({std::sin(phi)*.6,std::cos(phi)*.5,-1});
 tracker.observe(obs(d*9.80665,4.0,{-.08,.14,.09},i));}
 check(tracker.changes()>=1,"change detected");
 check(tracker.estimate().observable,"new load observable");
 close(tracker.estimate().mass_kg,4.0,.03,"load adaptation");
 check(norm(tracker.estimate().com_sensor_m-Vec3{-.08,.14,.09})<.01,"new com");
 // 10. No change should occur under one isolated force impulse.
 AdaptivePayloadTracker nofalse(3.0,4);
 for(int i=0;i<130;++i){double phi=i*2.4;Vec3 d=normalized({std::sin(phi)*.6,std::cos(phi)*.5,-1});auto v=obs(d*9.80665,2.0,{.05,0,.1},i);if(i==100)v.force.x+=30;
 nofalse.observe(v);}
 check(nofalse.changes()==0,"false change prevented");
 nofalse.observe(obs({0,0,-9.80665},2,{.05,0,.1},1));
 check(nofalse.dropped_inputs()==1,"replayed measurement dropped");
 // 11. No data and mass validation
 check(!PayloadEstimator{}.estimate().observable,"no data status");
 bool thrown=false;try{body_com({});}catch(const std::invalid_argument&){thrown=true;}check(thrown,"empty body fails");
 std::cout<<"11 core test groups passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}
