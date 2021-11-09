// Example program

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>

using namespace std;

double geomA = 0.035; //m distance between A and base
double geomB = 0.035; //m distance between B and mobile platform
double geomL0 = 0.2;  // arm lenght
double geomLg0 = 0.2;
double winchRadius = 0.0093;
double theta,phi, psi;
double R, phi1, phi2, phi3;



bool computeIk(double incl, double orien, std::vector<double> & lengths)
{
    theta=incl*M_PI/180;
    psi=orien*M_PI/180;

    //Pendiente
    // Calcular angulo de bloque y usar arco

      if (theta!=0)
      {
          R=geomL0/theta;

          phi1= M_PI/2-psi;

          phi2= 7*M_PI/6-psi;

          phi3= 11*M_PI/6-psi;

          lengths[0]=geomL0 - theta * geomA * cos(phi1);
          lengths[1]=geomL0 - theta * geomA * cos(phi2);
          lengths[2]=geomL0 - theta * geomA * cos(phi3);
      }
      else
      {
          lengths[0]=geomL0;
          lengths[1]=geomL0;
          lengths[2]=geomL0;
      }
    return true;
}
    
int main(int argc, char *argv[])
{
      double inc = atof(argv[1]);
      double ori = atof(argv[2]);

      cout << "running softArm IK solver: inclination: "<<inc<<", orientation: "<<ori<<endl;
      vector<double> v(3);
      vector<double> p(3);
      computeIk(inc,ori,v);
      cout <<"lenghts: "<<v[0] <<","<<v[1]<<","<<v[2]<<"\n";

      
      p[0] = geomLg0 - v[0];
      p[1] = geomLg0 - v[1];
      p[2] = geomLg0 - v[2];

      //p[0] = (geomLg0 - v[0]) / winchRadius;
      //p[1] = (geomLg0 - v[1]) / winchRadius;
      //p[2] = (geomLg0 - v[2]) / winchRadius;
      cout <<"angular motors: "<< p[0] <<","<<p[1]<<","<<p[2]<<"\n";
}

