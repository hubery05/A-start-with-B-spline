#include "bspline.h"


int main()
{
    double c_1[] = {-1.0, 1.0, 3.0, 3.0, 3.0, 3.0, 5.0, 6.0, 8.0, 9.0};
    double c_2[] = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0, 3.5, 5.0};
    int num = sizeof(c_1) / sizeof(c_1[0]);
    cout << "num:" << num <<endl;
    vector<double> x_set, y_set;
    for(int i=0;i<num;i++){
        x_set.push_back(c_1[i]);
        y_set.push_back(c_2[i]);
    }
    Points points;
    points = B_Spline(x_set,y_set);
    

    for(int i=0;i<points.points_x.size();i++){
        cout << points.points_x[i] << " ";
    }

    cout << "" << endl;
    for(int i=0;i<points.points_y.size();i++){
        cout << points.points_y[i] << " ";
    }


}
