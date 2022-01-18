// Example program to solve the kinematics in 2 different ways
// - Table form:        ./softArmKinematicSolver 10 90 ~/Tabla170.csv
// - Mathematical form: ./softArmKinematicSolver 10 90

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

double geomA = 0.035; //m distance between A and base
double geomB = 0.035; //m distance between B and mobile platform
double geomL0 = 0.2;  // arm lenght
double geomLg0 = 0.2;
double winchRadius = 0.0093;
double theta,phi, psi;
double R, phi1, phi2, phi3;

vector < vector<long> > lookupIndex;
vector < vector<double> > lookupTable;

/*******************
 *** Table solver **/
int initTableIk(string csvfileName, vector<int> tableDimensions)
{
    ifstream csv;
       csv.open(csvfileName);
       csv.seekg(0);

       string line;
       double i1,i2;
       double l1,l2,l3;

       getline(csv,line);
       lookupIndex.resize(tableDimensions[0]);

       for (int i=0; i<lookupIndex.size(); i++)
       {
           lookupIndex[i].resize(tableDimensions[1]);

           for (int j=0; j<lookupIndex[i].size(); j++)
           {
               //get line from file
               getline(csv,line);
               istringstream ss(line);

               //compare index
               ss >> i1;
               ss >> i2;
               ss >> l1;
               ss >> l2;
               ss >> l3;

               //store index
               lookupIndex[i][j]=lookupTable.size();
               //add another line to table
               lookupTable.push_back(vector<double>{l1,l2,l3});
           }
       }
       return true;
}



int readTableIk(double incl, double orien, std::vector<double> & lengths)
{
    //  cout << lookupIndex.size();
     long index = lookupIndex[incl][orien];
     if (lengths.size()!=lookupTable[index].size())
     {
         cout << "Wrong size." << endl;
         return -1;
     }

     lengths = lookupTable[index];

     return true;
}

/******************
 *** Math solver **/
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
    vector<double> l(3);
    vector<double> p(3);

    double inc = atof(argv[1]);
    double ori = atof(argv[2]);

      if (argc==4)
      {
          cout << "running softArm Table IK Solver at: "<< argv[3] << " with inc: "<<inc<<" ori: "<<ori<< endl;
          // table form
          bool ok = true;
          ok &= initTableIk(argv[3], vector<int>{171,360});
          ok &= readTableIk(inc, ori, l);

          if (!ok)
          {
              cout << "error: problems getting IK from CSV table" <<endl;
              return false;
          }
      }

      else
      {
          cout << "running softArm Math IK solver with inc: "<<inc<<" ori: "<<ori<< endl;
          computeIk(inc,ori,l);
      }

      cout <<"total lenghts: "<<l[0] <<","<<l[1]<<","<<l[2]<<"\n";

      
      p[0] = geomLg0 - l[0];
      p[1] = geomLg0 - l[1];
      p[2] = geomLg0 - l[2];

      cout <<"partial lenghts: "<< p[0] <<","<<p[1]<<","<<p[2]<<"\n";
}

