#include <iostream>
#include <vector>

using namespace std;

struct Points{
  vector<double> points_x;
  vector<double> points_y;
};

int k = 9;
int num;
vector<int> knots;


// Recursive computation of B-spline functions.
double Bspline(int index, int order, double u)
{
   double coef1, coef2;
   if ( order == 1 )
   {
	  if ( index == 0 ) if ( ( knots[index] <= u ) && ( u <= knots[index+1] ) ) return 1.0;
      if ( ( knots[index] < u ) && ( u <= knots[index+1] ) ) return 1.0;
	  else return 0.0;
   }
   else
   {
      if ( knots[index + order - 1] == knots[index] ) 
	  {
	     if ( u == knots[index] ) coef1 = 1;
		 else coef1 = 0;
	  }
	  else coef1 = (u - knots[index])/(knots[index + order - 1] - knots[index]);

      if ( knots[index + order] == knots[index+1] )
	  {
		 if ( u == knots[index + order] ) coef2 = 1;
		 else coef2 = 0;
	  }
	  else coef2 = (knots[index + order] - u)/(knots[index + order] - knots[index+1]);
		
      return ( coef1 * Bspline(index, order-1, u) + coef2 * Bspline(index+1,order-1 ,u) );
   }
}

double bspline(int num, double u, vector<double> c, int k)
{
    double C = 0;
    for (int i=0;i < num;i++){
       C += c[i] * Bspline(i, k, u);
       //cout << "B[" << i << "]:" << Bspline(i, k, u) << endl;
    }
    return C;
}

Points B_Spline(vector<double> x_set, vector<double> y_set)
{
    int num = x_set.size();
    
    for(int i=0;i<=num+k;i++){
      if(i <= k){
        knots.push_back(0);
      }
      else if(i >= num){
        knots.push_back(num-k);
      }
      else{
        knots.push_back(i-k);
      }
    }
    //for(int i=0;i<knots.size();i++){
    //    cout << "knots[" << knots[i]  << "]" << " ";
    //}
    
    Points points_;
    for(float i=0.0;i<=num-k;i=i+0.5){
        double x_ = bspline(num,i,x_set,k);
        double y_ = bspline(num,i,y_set,k);
        points_.points_x.push_back(x_);
        points_.points_y.push_back(y_);
    }

    return points_;
}
