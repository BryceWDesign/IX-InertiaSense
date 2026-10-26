// SPDX-License-Identifier: Apache-2.0
#include "ix_inertiasense/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ix {
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double norm(Vec3 v){return std::sqrt(dot(v,v));}
bool finite(Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
Vec3 normalized(Vec3 v){double n=norm(v); if(n<1e-12) throw std::invalid_argument("zero vector");return v/n;}
Quaternion Quaternion::operator*(Quaternion b) const {
 return {w*b.w-x*b.x-y*b.y-z*b.z,w*b.x+x*b.w+y*b.z-z*b.y,w*b.y-x*b.z+y*b.w+z*b.x,w*b.z+x*b.y-y*b.x+z*b.w};
}
Quaternion Quaternion::unit() const {
 double n=std::sqrt(w*w+x*x+y*y+z*z);
 if(!std::isfinite(n)||n<1e-12) throw std::invalid_argument("invalid quaternion");
 return {w/n,x/n,y/n,z/n};
}
Vec3 Quaternion::rotate(Vec3 v) const {
 Quaternion r=(*this)*Quaternion{0,v.x,v.y,v.z}*conjugate();return {r.x,r.y,r.z};
}
void VestibularFilter::reset(Quaternion q){q_=q.unit();rejected_=0;}
bool VestibularFilter::update(Vec3 gyro,Vec3 specific_force,double dt){
 if(!finite(gyro)||!finite(specific_force)||!std::isfinite(dt)||dt<=0||dt>0.1) return false;
 // Integrate body-frame gyro, then correct tilt using gravity ONLY if specific
 // force magnitude is consistent with static gravity. Yaw remains unobservable.
 double omega=norm(gyro), angle=omega*dt;
 Quaternion dq;
 if(angle>1e-10){double k=std::sin(angle/2)/omega; dq={std::cos(angle/2),gyro.x*k,gyro.y*k,gyro.z*k};}
 else dq={1,gyro.x*dt/2,gyro.y*dt/2,gyro.z*dt/2};
 q_=(q_*dq).unit();
 double a=norm(specific_force);
 if(a<8.8||a>10.8){++rejected_;return true;}
 Vec3 observed_up=normalized(specific_force);
 Vec3 expected_up=q_.conjugate().rotate({0,0,1});
 Vec3 error=cross(observed_up,expected_up);
 // correction must rotate q toward observed upright in body coordinates
 double en=norm(error); if(en>1e-12){
  double corr=std::min(gain_*en,0.2), s=std::sin(corr/2)/en;
  Quaternion delta{std::cos(corr/2),error.x*s,error.y*s,error.z*s};
  q_=(q_*delta).unit();
 }
 return true;
}
namespace {
using M4=std::array<std::array<double,4>,4>;
using V4=std::array<double,4>;
// Wrench model: F = m*g; torque = h x g, h=m*r
void rows(const WrenchObservation& s,std::array<V4,6>& a,std::array<double,6>& y){
 Vec3 g=s.gravity;
 a={V4{g.x,0,0,0},V4{g.y,0,0,0},V4{g.z,0,0,0},
    V4{0,0,g.z,-g.y},V4{0,-g.z,0,g.x},V4{0,g.y,-g.x,0}};
 y={s.force.x,s.force.y,s.force.z,s.torque.x,s.torque.y,s.torque.z};
}
void accumulate(const WrenchObservation& s,double w,M4& H,V4& b){
 std::array<V4,6>a{};std::array<double,6>y{};rows(s,a,y);
 for(int k=0;k<6;++k){double weight=w*(k<3?1:16);for(int i=0;i<4;++i){
 b[i]+=weight*a[k][i]*y[k];for(int j=0;j<4;++j) H[i][j]+=weight*a[k][i]*a[k][j];}}
}
V4 solve(M4 H,V4 b){
 for(int i=0;i<4;++i){int p=i;for(int j=i+1;j<4;++j)if(std::abs(H[j][i])>std::abs(H[p][i]))p=j;
 if(std::abs(H[p][i])<1e-13)throw std::runtime_error("rank deficient");
 std::swap(H[i],H[p]);std::swap(b[i],b[p]);
 double d=H[i][i];for(int j=i;j<4;++j)H[i][j]/=d;b[i]/=d;
 for(int k=0;k<4;++k)if(k!=i){double f=H[k][i];for(int j=i;j<4;++j)H[k][j]-=f*H[i][j]; b[k]-=f*b[i];}
 }
 return b;
}
std::array<double,4> eig4(M4 A){
 for(int it=0;it<100;++it){int p=0,q=1;double max=0;
  for(int i=0;i<4;++i)for(int j=i+1;j<4;++j)if(std::abs(A[i][j])>max){max=std::abs(A[i][j]);p=i;q=j;}
  if(max<1e-10)break;
  double phi=0.5*std::atan2(2*A[p][q],A[q][q]-A[p][p]);
  double c=std::cos(phi),s=std::sin(phi);
  double app=A[p][p],aqq=A[q][q],apq=A[p][q];
  A[p][p]=c*c*app-2*s*c*apq+s*s*aqq;
  A[q][q]=s*s*app+2*s*c*apq+c*c*aqq;
  A[p][q]=A[q][p]=0;
  for(int k=0;k<4;++k)if(k!=p&&k!=q){double x=A[k][p],y=A[k][q];A[k][p]=A[p][k]=c*x-s*y;A[k][q]=A[q][k]=s*x+c*y;}
 }
 return {A[0][0],A[1][1],A[2][2],A[3][3]};
}
M4 information(const std::deque<WrenchObservation>& v){M4 H{};V4 b{};for(auto&s:v)accumulate(s,s.quality,H,b);return H;}
}
PayloadEstimator::PayloadEstimator(std::size_t window,double max_extent_m):window_(window),extent_(max_extent_m){
 if(window<4||!std::isfinite(extent_)||extent_<=0)throw std::invalid_argument("invalid estimator configuration");
}
void PayloadEstimator::clear(){samples_.clear();rejected_=0;last_time_=-std::numeric_limits<double>::infinity();}
bool PayloadEstimator::add(const WrenchObservation& s){
 double g=norm(s.gravity); if(!s.healthy||!s.static_equilibrium||!finite(s.gravity)||!finite(s.force)||!finite(s.torque)||!std::isfinite(s.timestamp_s)||s.timestamp_s<=last_time_||!std::isfinite(s.quality)||s.quality<=0||s.quality>1||g<9.0||g>10.7){++rejected_;return false;}
 samples_.push_back(s);last_time_=s.timestamp_s;if(samples_.size()>window_)samples_.pop_front();return true;
}
PayloadEstimate PayloadEstimator::estimate()const{
 PayloadEstimate o;o.sample_count=samples_.size();o.rejected=rejected_;
 if(samples_.size()<4){o.status="insufficient_samples";return o;}
 M4 H=information(samples_);auto eig=eig4(H);
 double lo=*std::min_element(eig.begin(),eig.end()),hi=*std::max_element(eig.begin(),eig.end());
 o.min_information_eigenvalue=lo;
 if(lo<=1e-7||lo/hi<1e-4){o.status="unobservable_gravity_directions";return o;}
 o.max_information_condition=hi/lo;
 V4 x{};
 for(int it=0;it<9;++it){M4 A{};V4 b{};
 for(auto &s:samples_){std::array<V4,6> a{};std::array<double,6> y{};rows(s,a,y);
 double ss=0;for(int k=0;k<6;++k){double pred=0;for(int j=0;j<4;++j)pred+=a[k][j]*x[j];double d=(pred-y[k])*(k<3?1:4);ss+=d*d;}
 double residual=std::sqrt(ss);double w=s.quality*std::min(1.0,2.0/std::max(residual,1e-12));accumulate(s,w,A,b);
 }
 try{x=solve(A,b);}catch(const std::exception&){o.status="numerical_singularity";return o;}
 }
 if(!std::isfinite(x[0])||x[0]<0.01){o.status="nonphysical_mass";return o;}
 Vec3 r{x[1]/x[0],x[2]/x[0],x[3]/x[0]};
 if(!finite(r)||norm(r)>extent_){o.status="outside_payload_bounds";return o;}
 double sum=0,sw=0;
 for(auto&s:samples_){Vec3 f=s.gravity*x[0],t=cross(Vec3{x[1],x[2],x[3]},s.gravity);
 double residual=dot(s.force-f,s.force-f)+16*dot(s.torque-t,s.torque-t);
 sum+=s.quality*residual;sw+=s.quality;}
 o.weighted_rms=std::sqrt(sum/sw);o.mass_kg=x[0];o.com_sensor_m=r;o.observable=true;o.status="estimated_quasistatic";return o;
}
int PayloadEstimator::suggest_probe(const std::vector<Vec3>& gravity_candidates)const{
 int best=-1;double best_value=-std::numeric_limits<double>::infinity();
 M4 H=information(samples_);
 for(std::size_t i=0;i<gravity_candidates.size();++i){auto g=gravity_candidates[i];if(!finite(g)||norm(g)<9.0||norm(g)>10.7)continue;
  M4 next=H;V4 b{};accumulate({g,{},{},0,true,true,1.0},1.0,next,b);
  auto e=eig4(next);double score=*std::min_element(e.begin(),e.end());
  if(score>best_value+1e-8){best_value=score;best=static_cast<int>(i);}
 }
 return best;
}
AdaptivePayloadTracker::AdaptivePayloadTracker(double threshold,unsigned hits):threshold_(threshold),required_(hits){
if(!std::isfinite(threshold)||threshold<=0||hits<2)throw std::invalid_argument("invalid tracker configuration");
}
void AdaptivePayloadTracker::observe(const WrenchObservation& s){
 double g=norm(s.gravity);
 if(!s.healthy||!s.static_equilibrium||!finite(s.gravity)||!finite(s.force)||!finite(s.torque)||!std::isfinite(s.timestamp_s)||s.timestamp_s<=last_seen_||!std::isfinite(s.quality)||s.quality<=0||s.quality>1||g<9.0||g>10.7){++dropped_;return;}
 last_seen_=s.timestamp_s;
 auto prev=estimator_.estimate();
 if(prev.observable && s.static_equilibrium && s.healthy && finite(s.gravity) && finite(s.force) && finite(s.torque)){
  Vec3 pf=s.gravity*prev.mass_kg;
  Vec3 pt=cross(prev.com_sensor_m*prev.mass_kg,s.gravity);
  double res=std::sqrt(dot(s.force-pf,s.force-pf)+16*dot(s.torque-pt,s.torque-pt));
  if(res>threshold_){
   ++run_;suspected_.push_back(s);
   if(run_>=required_){
    estimator_.clear();
    for(const auto& candidate:suspected_)estimator_.add(candidate);
    ++changes_;run_=0;suspected_.clear();
   }
   return;
  }
 }
 // A healthy normal sample breaks the mismatch streak; contamination may be
 // isolated, so do not poison the model with unresolved suspect samples.
 run_=0;suspected_.clear();estimator_.add(s);
}
Vec3 body_com(const std::vector<BodySegment>& parts){double sum=0;Vec3 h{};for(auto &p:parts){if(!std::isfinite(p.mass_kg)||p.mass_kg<=0||!finite(p.com_world_m))throw std::invalid_argument("bad link");sum+=p.mass_kg;h=h+p.com_world_m*p.mass_kg;}if(sum==0)throw std::invalid_argument("no segments");return h/sum;}
Vec3 loaded_com(const std::vector<BodySegment>& parts,double m,Vec3 pos){if(m<0||!std::isfinite(m)||!finite(pos))throw std::invalid_argument("bad payload");double total=m;Vec3 moment=pos*m;for(auto &p:parts){if(p.mass_kg<=0||!std::isfinite(p.mass_kg)||!finite(p.com_world_m))throw std::invalid_argument("bad segment");total+=p.mass_kg;moment=moment+p.com_world_m*p.mass_kg;}if(total<=0)throw std::invalid_argument("empty body");return moment/total;}
double signed_support_margin(const std::vector<Vec2>& poly,Vec2 p){
 if(poly.size()<3||!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("invalid polygon or point");
 double min=std::numeric_limits<double>::infinity();
 for(std::size_t i=0;i<poly.size();++i){
  auto a=poly[i],b=poly[(i+1)%poly.size()],c=poly[(i+2)%poly.size()];
  if(!std::isfinite(a.x)||!std::isfinite(a.y))throw std::invalid_argument("nonfinite polygon");
  double dx=b.x-a.x,dy=b.y-a.y;double len=std::hypot(dx,dy);
  if(len<1e-12)throw std::invalid_argument("degenerate edge");
  double turn=dx*(c.y-b.y)-dy*(c.x-b.x);
  if(turn<=1e-12)throw std::invalid_argument("polygon must be strictly convex CCW");
  double d=(dx*(p.y-a.y)-dy*(p.x-a.x))/len;min=std::min(min,d);
 }return min;
}
Vec2 capture_point(Vec2 p,Vec2 v,double h,double g){if(h<=0||g<=0||!std::isfinite(h)||!std::isfinite(g))throw std::invalid_argument("invalid height or gravity");double t=std::sqrt(h/g);return {p.x+t*v.x,p.y+t*v.y};}
}
