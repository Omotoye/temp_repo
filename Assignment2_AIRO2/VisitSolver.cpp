    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include "armadillo"
#include <initializer_list>
#include "Eigen/Dense"

using namespace std;
using namespace arma;
using Eigen::MatrixXd;
using Eigen::VectorXd;



    //map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){

}

VisitSolver::~VisitSolver(){

}

void VisitSolver::loadSolver(string *parameters, int n){
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/root/AI_assign2/popf-tif/planner/release/popf/visits_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/root/AI_assign2/popf-tif/planner/release/popf/visits_domain/landmark.txt";
  parseLandmark(landmark_file);


        //startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;
  double euc_cost_result=0;
  double kalman_cost_result=0;
  double result=0;

  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);
      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

      string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
      string to = tmp.substr(3,2);
      

     //**ADDED CODE***//
     euc_cost_result = calculateEuclidean(from, to);
     kalman_cost_result = calculateKalman(from, to);
     result= euc_cost_result + kalman_cost_result;
     cout<<"\n Euclidean Cost: "<<euc_cost_result<<endl;
     cout<<"\n Kalman Filter Cost: "<<kalman_cost_result<<endl;
     //*****//

    }
  }
}else{
  if(function=="dummy"){
    dummy = value;

  }else if(function=="act-cost"){
    act_cost = value;
    
                 } //else if(function=="dummy1"){
                    //duy = value;              
                    ////cout << parameter << " " << value << endl;
                 //}
                 }
               
               }


               //double results = calculateExtern(dummy, act_cost);
	       
			   
			   
			   if (ExternalSolver::verbose){
                //cout << "(dummy) " << results << endl;
				cout << "(dummy) " << result << endl;
              }

              //toReturn["(dummy)"] = results;
			  toReturn["(dummy)"] = result;


              return toReturn;
            }

            list<string> VisitSolver::getParameters(){

              return affected;
            }

            list<string> VisitSolver::getDependencies(){

              return dependencies;
            }
			
/*
         double VisitSolver::calculateExtern(double external, double total_cost){
       //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
       double cost = 2;//random1;
       return cost;
     }
*/
            
			//** ADDED FUNCTION ***//
                        // Compute Euclidean cost between two regions //
			double VisitSolver::calculateEuclidean(string from, string to){

				double from_pose_x, from_pose_y, from_pose_th;
				double to_pose_x, to_pose_y, to_pose_th;
				string from_waypoint, to_waypoint;
				double SqrEucCost, EucCost;
				
                                //get each x,y, and theta values of the start and end positions
				from_waypoint = region_mapping[from.c_str()][0];
				to_waypoint = region_mapping[to.c_str()][0];
				from_pose_x = waypoint[from_waypoint.c_str()][0];
				from_pose_y = waypoint[from_waypoint.c_str()][1];
				from_pose_th = waypoint[from_waypoint.c_str()][2];
				to_pose_x = waypoint[to_waypoint.c_str()][0];
				to_pose_y = waypoint[to_waypoint.c_str()][1];
				to_pose_th = waypoint[to_waypoint.c_str()][2];

                                
				//Calculate the euclidean distance between start and end positions
				SqrEucCost = pow(to_pose_x - from_pose_x, 2) + pow(to_pose_y - from_pose_y, 2) + pow(to_pose_th - from_pose_th, 2);
				EucCost = sqrt(SqrEucCost);
                                cout<<"\nfrom pose "<<from_pose_x<<" "<<from_pose_y<<" "<<from_pose_th<<endl;
                                cout<<"to pose "<<to_pose_x<<" "<<to_pose_y<<" "<<to_pose_th<<endl;
                                
				return EucCost;
				
			}
			

                        //** ADDED FUNCTION ***//
                        // Compute Localization cost between two regions using EKF //
			double VisitSolver::calculateKalman(string from, string to){
                                X = MatrixXd(3,1);
                                A = MatrixXd(3, 3);
                                P = MatrixXd(3, 3);
                                C = MatrixXd(2, 3);
                                Q_gamma = MatrixXd(2, 2);
                                K = MatrixXd(3, 2);

                                double delta_D, delta_Theta;     //real input values 
                                double delta_D_m, delta_Theta_m; //measured input values with encoders

                                
                                double KalCost=0;
                                
				double from_pose_x, from_pose_y, from_pose_th;
				double to_pose_x, to_pose_y, to_pose_th;
				string from_waypoint, to_waypoint;
                                
				
				from_waypoint = region_mapping[from.c_str()][0];
				to_waypoint = region_mapping[to.c_str()][0];
				from_pose_x = waypoint[from_waypoint.c_str()][0];
				from_pose_y = waypoint[from_waypoint.c_str()][1];
				from_pose_th = waypoint[from_waypoint.c_str()][2];
				to_pose_x = waypoint[to_waypoint.c_str()][0];
				to_pose_y = waypoint[to_waypoint.c_str()][1];
				to_pose_th = waypoint[to_waypoint.c_str()][2];
                                
                                double SqrEucDis = pow(to_pose_x - from_pose_x, 2) + pow(to_pose_y - from_pose_y, 2);

				delta_D = sqrt(SqrEucDis);
                                delta_Theta = to_pose_th - from_pose_th;

                                float r1,r2;  //Random float numbers (from 0 to 1) to be added to the input values
                                r1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                                r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                                
                                delta_D_m = delta_D + r1;
                                delta_Theta_m = delta_Theta + (0.25*r2);

                                cout<<"\ndelta_D_real "<< delta_D <<" delta_Theta_real "<< delta_Theta << endl;
                                cout<<"\ndelta_D_measured "<< delta_D_m <<" delta_Theta_measured "<< delta_Theta_m << endl;
                                
                                
                                
                                double v1 = -delta_D_m*sin(X(2,0)+(delta_Theta_m/2));
                                double v2 = delta_D_m*cos(X(2,0)+(delta_Theta_m/2));
                                A << 1, 0, v1,
                                      0, 1, v2,
                                      0, 0, 1;
                                                                

                                //initial covariance matrix as stated in the assignment document
                                P << 0.02, 0, 0,
                                      0, 0.02, 0,
                                      0, 0, 0.02;

                                //// PREDICTION PHASE /////
                                X << from_pose_x + delta_D_m*cos(X(2,0)+(delta_Theta_m/2)),
                                      from_pose_y + delta_D_m*sin(X(2,0)+(delta_Theta_m/2)),
                                      X(2,0)+delta_Theta_m;

                                P = A*P*A.transpose();


                                //// ESTIMATION PHASE /////
                                vector<double> landmark1, landmark2;
                                landmark1 = landmark["l1"];
                                landmark2 = landmark["l2"];

                                //Check if the robot has passed over a landmark 
                                //Meaning that we check if the start and end points are collinear with each of the landmarks
                                vector<double> p_landmark;
                                double collinear1, collinear2;
                                collinear1 = 0.5*(((from_pose_x - to_pose_x)*(to_pose_y - landmark1[1])) - ((to_pose_x - landmark1[0])*(from_pose_y - to_pose_y)));
                                collinear2 = 0.5*(((from_pose_x - to_pose_x)*(to_pose_y - landmark2[1])) - ((to_pose_x - landmark2[0])*(from_pose_y - to_pose_y)));
                                if(collinear1 == 0){
                                    p_landmark = landmark1;
                                    cout<<"\nLandmark1 detected"<<endl;
                                }
                                else if(collinear2 == 0){
                                    p_landmark = landmark2;
                                    cout<<"\nLandmark2 detected"<<endl;
                                }
                                else{
                                   //The robot hasn't passed over a landmark. So, no estimation phase.
                                   KalCost = P(0,0)+P(1,1)+P(2,2);
                                   cout<<"\nNo landmark detected "<<endl;
                                   return KalCost;
                                }

                                //measurement covariance matrix of white gaussian noise from -1 to 1
                                Q_gamma << 0.1, 0,
                                      0, 0.1;

                                MatrixXd I = MatrixXd::Identity(3, 3);
                                C << 2*(X(0,0)-p_landmark[0]), 2*(X(1,0)-p_landmark[1]), 0,
                                      0, 0, 1;

                                MatrixXd Ct = C.transpose();
                                MatrixXd PCt = P*Ct;
                                MatrixXd F = C*PCt + Q_gamma;
                                K = PCt * F.inverse();
                                MatrixXd IKC = I - (K*C);
                                P = IKC * P;
                                KalCost = P(0,0)+P(1,1)+P(2,2);


                                
				return KalCost;
				
			}
			//*****//
                        /*//MatrixXd G(2,1);
                                //double g1 = pow((X(0,0)-p_landmark[0]),2)+ pow((X(1,0)-p_landmark[1]),2);
                                //double g2 = X(2,0) - p_landmark[2];
                                //G << g1,
                                //     g2;

                                //double y1 = pow((to_pose_x - p_landmark[0]),2)+ pow((to_pose_y-p_landmark[1]),2);
                                //double y2 = to_pose_th - p_landmark[2];
                                //Y << y1,
                                //     y2;*/
			
			void VisitSolver::parseParameters(string parameters){

              int curr, next;
              string line;
              ifstream parametersFile(parameters.c_str());
              if (parametersFile.is_open()){
                while (getline(parametersFile,line)){
                 curr=line.find(" ");
                 string region_name = line.substr(0,curr).c_str();
                 curr=curr+1;
                 while(true ){
                  next=line.find(" ",curr);
                  region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
                  if (next ==-1)
                   break;
                 curr=next+1;

               }                
             }

           }

         }


     void VisitSolver::parseWaypoint(string waypoint_file){

       int curr, next;
       string line;
       double pose1, pose2, pose3;
       ifstream parametersFile(waypoint_file);
       if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
         curr=line.find("[");
         string waypoint_name = line.substr(0,curr).c_str();

         curr=curr+1;
         next=line.find(",",curr);

         pose1 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find(",",curr);

         pose2 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find("]",curr);

         pose3 = (double)atof(line.substr(curr,next-curr).c_str());

         waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
       }
     }

   }

   void VisitSolver::parseLandmark(string landmark_file){

     int curr, next;
     string line;
     double pose1, pose2, pose3;
     ifstream parametersFile(landmark_file);
     if (parametersFile.is_open()){
      while (getline(parametersFile,line)){
       curr=line.find("[");
       string landmark_name = line.substr(0,curr).c_str();
       
       curr=curr+1;
       next=line.find(",",curr);

       pose1 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find(",",curr);

       pose2 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find("]",curr);

       pose3 = (double)atof(line.substr(curr,next-curr).c_str());

       landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
     }
   }
   
 }


  //void VisitSolver::localize( string from, string to){
  //} 







