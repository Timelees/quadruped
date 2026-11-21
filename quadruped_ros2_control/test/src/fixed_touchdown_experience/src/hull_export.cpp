// Export exact production hulls of every admitted replay candidate, without
// changing its curve or choosing a shape to obtain an attractive comparison.
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "ocs2_quadruped_controller/interface/constraint/PolynomialHull.h"
using namespace ocs2::legged_robot;
void matrix(std::ostream& f,const char* name,const Eigen::MatrixXd& m){
 f<<",\""<<name<<"\":[";for(int i=0;i<m.rows();++i){f<<(i?",":"")<<'[';
 for(int j=0;j<m.cols();++j)f<<(j?",":"")<<m(i,j);f<<']';}f<<']';
}
int main(int argc,char**argv){
 if(argc!=3){std::cerr<<"usage: hull_export ACCEPTED_JSONL OUTPUT_JSONL\n";return 2;}
 if(std::ifstream(argv[2]).good())throw std::runtime_error("output exists");
 std::ifstream input(argv[1]);std::ofstream out(argv[2]);
 if(!input || !out)throw std::runtime_error("cannot open input/output");
 out<<std::setprecision(17);
 std::string line;int parent=0;
 while(std::getline(input,line)){
  boost::property_tree::ptree pt;std::istringstream buf(line);boost::property_tree::read_json(buf,pt);
  auto& ref=pt.get_child("new");std::vector<double> kv;std::vector<Eigen::Vector3d> cv;
  for(const auto& v:ref.get_child("knots"))kv.push_back(v.second.get_value<double>());
  for(const auto& r:ref.get_child("controls")){Eigen::Vector3d c;int k=0;for(const auto& v:r.second)c[k++]=v.second.get_value<double>();cv.push_back(c);}
  BSpline s(5);Eigen::VectorXd x(cv.size()),y(cv.size()),z(cv.size());
  for(size_t i=0;i<cv.size();++i){x[i]=cv[i].x();y[i]=cv[i].y();z[i]=cv[i].z();}
  s.setKnots(Eigen::Map<Eigen::VectorXd>(kv.data(),kv.size()));s.setControlPoints(x,y,z);
  for(const auto& seg:extractPolynomialSegments(s)){
   const Eigen::MatrixXd b=hullMap(seg,HullBasis::Bezier)*s.getControlPoints();
   out<<"{\"source\":"<<10000+parent<<",\"parent_index\":"<<parent<<",\"start\":"<<seg.start<<",\"end\":"<<seg.end;
   matrix(out,"bernstein",b);matrix(out,"minvo",hullMap(seg,HullBasis::Minvo)*s.getControlPoints());
   out<<",\"spline\":{\"knots\":[";for(int k=0;k<12;++k)out<<(k?",":"")<<(k<6?seg.start:seg.end);
   out<<']';matrix(out,"controls",b);out<<"}}\n";
  }++parent;
 }
 std::cout<<"curves="<<parent<<'\n';
}
