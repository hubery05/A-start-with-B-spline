#include<iostream>
#include<vector>
#include<array>
#include<string>
#include<Eigen/Eigen>
#include<stdexcept>

using namespace std;

vector<double> vec_diff(vector<double> input){
  vector<double> output;
  for(unsigned int i=1; i<input.size(); i++){
    output.push_back(input[i] - input[i-1]);
  }
  return output;
};

vector<double> cum_sum(vector<double> input){
  vector<double> output;
  float temp = 0;
  for(unsigned int i=0; i<input.size(); i++){
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

class Spline{
public:
  vector<double> x;
  vector<double> y;
  int nx;
  vector<double> h;
  vector<double> a;
  vector<double> b;
  vector<double> c;
  //Eigen::VectorXf c;
  vector<double> d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(vector<double> x_, vector<double> y_):x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_){
    Eigen::MatrixXf A = calc_A();
    Eigen::VectorXf B = calc_B();
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    float * c_pointer = c_eigen.data();
    //Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
    c.assign(c_pointer, c_pointer+c_eigen.rows());

    for(int i=0; i<nx-1; i++){
      d.push_back((c[i+1]-c[i])/(3.0*h[i]));
      b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
    }
  };

  float calc(float t){
    if(t<x.front() || t>x.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    float dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  };

private:
  Eigen::MatrixXf calc_A(){
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
    A(0, 0) = 1;
    for(int i=0; i<nx-1; i++){
      if (i != nx-2){
        A(i+1, i+1) = 2 * (h[i] + h[i+1]);
      }
      A(i+1, i) = h[i];
      A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx-1, nx-2) = 0.0;
    A(nx-1, nx-1) = 1.0;
    return A;
  };
  Eigen::VectorXf calc_B(){
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
    for(int i=0; i<nx-2; i++){
      B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
    }
    return B;
  };

  int bisect(float t, int start, int end){
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1){
      return mid;
    }else if (t>x[mid]){
      return bisect(t, mid, end);
    }else{
      return bisect(t, start, mid);
    }
  }
};

class Spline2D{
public:
  Spline sx;
  Spline sy;
  vector<double> s;

  Spline2D(vector<double> x, vector<double> y){
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };

  array<float, 2> calc_postion(float s_t){
    float x = sx.calc(s_t);
    float y = sy.calc(s_t);
    return {{x, y}};
  };


private:
  vector<double> calc_s(vector<double> x, vector<double> y){
    vector<double> ds;
    vector<double> out_s{0};
    vector<double> dx = vec_diff(x);
    vector<double> dy = vec_diff(y);

    for(unsigned int i=0; i<dx.size(); i++){
      ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }

    vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};
