// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftArmControl.hpp"

#include <cmath>

#include <Eigen/Dense>

using namespace sofia;

// -----------------------------------------------------------------------------

void SoftArmControl::computeIk(double incl, double orien, std::vector<double> & lengths)
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

    //  cout<<A<<endl;
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
}

// -----------------------------------------------------------------------------

int SoftArmControl::initTableIk(string csvfileName)
{

    ifstream csv;
    csv.open(csvfileName);
    csv.seekg(0);

    string line;
    double i1,i2;
    double l1,l2,l3;

    vector<int> tableDimensions{41,360};

    getline(csv,line);


    lookupIndex.resize(tableDimensions[0]);
    for (int i=1; i<lookupIndex.size(); i++)
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

            if( (i1!=i) | (i2!=j) )
            {
                //cout << "line : " << line;
                cout << "index : ";
                cout << "i " << i << ", i1 " << i1 << ", j " << j << ", i2 " << i2 << endl;
            }
            //compression
            ss >> line;
            //and the values
            ss >> l1;
            ss >> l2;
            ss >> l3;


            //store index
            lookupIndex[i][j]=lookupTable.size();
            //add another line to table
            lookupTable.push_back(vector<double>{l1,l2,l3});
        }
    }

    return 0;

}

// -----------------------------------------------------------------------------

int SoftArmControl::readTableIk(double incl, double orien, std::vector<double> & lengths)
{
    //  cout << lookupIndex.size();
       long index = lookupIndex[theta][phi];
       if (lengths.size()!=lookupTable[index].size())
       {
           cout << "Wrong size." << endl;
           return -1;
       }

       lengths = lookupTable[index];

       return 0;
}
