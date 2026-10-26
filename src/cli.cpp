// SPDX-License-Identifier: Apache-2.0
#include "ix_inertiasense/core.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
using namespace ix;
int main(int argc,char**argv){
 if(argc!=3||(std::string(argv[1])!="fit"&&std::string(argv[1])!="track")){
  std::cerr<<"Usage: inertiasense fit|track WRENCH.csv\nCSV columns: time,gx,gy,gz,fx,fy,fz,tx,ty,tz,quality,static,healthy\n";return 2;
 }
 try{
 std::ifstream f(argv[2]);if(!f)throw std::runtime_error("unable to read file");
 PayloadEstimator estimator(512,1.0);
 AdaptivePayloadTracker tracker(4.0,5);
 bool track=std::string(argv[1])=="track";
 int first_change_line=-1;
 std::string row;int line=0;
 while(std::getline(f,row)){
  ++line;if(row.empty()||row[0]=='#'||row.rfind("time,",0)==0)continue;
  std::istringstream ss(row);std::vector<double> a;std::string col;
  while(std::getline(ss,col,',')){col.erase(col.find_last_not_of(" \t\r\n")+1);std::size_t consumed=0;double x=std::stod(col,&consumed);if(consumed!=col.size())throw std::runtime_error("invalid value");a.push_back(x);}
  if(a.size()!=13)throw std::runtime_error("wrong number of columns at line "+std::to_string(line));
  WrenchObservation s{{a[1],a[2],a[3]},{a[4],a[5],a[6]},{a[7],a[8],a[9]},a[0],a[11]!=0,a[12]!=0,a[10]};
  if(track){std::size_t prev=tracker.changes();tracker.observe(s);if(tracker.changes()>prev&&first_change_line<0)first_change_line=line;}
  else estimator.add(s);
 }
 auto e=track?tracker.estimate():estimator.estimate();std::cout<<std::fixed<<std::setprecision(9);
 std::cout<<"{\"status\":\""<<e.status<<"\",\"observable\":"<<(e.observable?"true":"false")
          <<",\"sample_count\":"<<e.sample_count<<",\"rejected\":"<<e.rejected;
 if(track)std::cout<<",\"change_events\":"<<tracker.changes()<<",\"first_change_line\":"<<first_change_line<<",\"dropped_inputs\":"<<tracker.dropped_inputs();
 if(e.observable)std::cout<<",\"mass_kg\":"<<e.mass_kg<<",\"com_sensor_m\":["<<e.com_sensor_m.x<<","<<e.com_sensor_m.y<<","<<e.com_sensor_m.z<<"],\"weighted_rms\":"<<e.weighted_rms<<",\"condition\":"<<e.max_information_condition;
 std::cout<<"}\n";return e.observable?0:3;
 }catch(const std::exception& e){std::cerr<<"error: "<<e.what()<<"\n";return 4;}
}
