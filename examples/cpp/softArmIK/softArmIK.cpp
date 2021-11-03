// Example program

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

double geomA;
double geomB;
double geomL0;
double geomLg0;
double winchRadius;
double theta,phi;
double t11, t12, t13, t21, t22, t23, t31, t32, t33, R, s0, t0, P, L;



bool computeIk(double incl, double orien, std::vector<double> & lengths)
{
    //    if (incl == 0) return -1;
        theta=incl*M_PI/180;
        if (incl == 0) theta=0.001*M_PI/180;
        phi=orien*M_PI/180;

       //Matrix A
        MatrixXd A(4,3);
      A <<  0, -sqrt(3)*geomA/2, sqrt(3)*geomA/2,
            geomA,       -0.5*geomA,  -0.5*geomA,
            0,            0,           0,
            1,            1,           1;


      //Matrix B
      MatrixXd B(4,3);
      B <<  0,      -sqrt(3)*geomB/2, sqrt(3)*geomB/2,
            geomB,       -0.5*geomB,  -0.5*geomB,
            0,            0,           0,
            1,            1,           1;

      //Matrix R
      MatrixXd R(3,3);
      t11 = pow(sin(phi),2)+cos(theta)*pow(cos(phi),2);
      t12=(cos(theta)-1)*cos(phi)*sin(phi);
      t21=t12;
      t13=sin(theta)*cos(phi);
      t31=-t13;
      t23=sin(theta)*sin(phi);
      t32=-t23;
      t22=pow(cos(phi),2)+cos(theta)*pow(sin(phi),2);
      t33=cos(theta);

     R <<  t11, t12, t13,
           t21, t22, t23,
           t31, t32, t33;

     //s0 and t0
     s0=geomL0*(1-cos(theta))/theta;
     t0=geomL0*sin(theta)/theta;



     //Matrix P traslation
     MatrixXd P(3,1);
     P<<   s0*cos(phi),
           s0*sin(phi),
                    t0;


     //Matrix T Homogenia
     MatrixXd T(4,4);
     T << R, P,
          0, 0, 0, 1;

     //Matrix length
     MatrixXd L(4,3);
     L= T*B-A;

     //Total Length
     lengths[0]=sqrt(pow(L(0,0),2)+pow(L(1,0),2)+pow(L(2,0),2));//L1
     lengths[1]=sqrt(pow(L(0,1),2)+pow(L(1,1),2)+pow(L(2,1),2));//L2
     lengths[2]=sqrt(pow(L(0,2),2)+pow(L(1,2),2)+pow(L(2,2),2));//L3

     return true;
}
    
int main()
{
  cout << "running softArm IK solver";
  vector<double> length;
  computeIk(5,2,length);
  cout << length[0] <<","<<length[1]<<","<<length[2]<<"\n";
}

