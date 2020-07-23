#include <iostream>
#include <vector>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include "NAOKinematics.h"
#include "KMat.hpp"
#include <fstream>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "KinematicsDefines.h"
#include "KinematicsDefines.h"

//Created by noobswestand github : NAaoPythonIK and modified by my_name_id_D for my ROS situation.
//CAN BE ADAPTED FOR LEFT AND RIGHT ARM
/*TAKING OBJECT POSE FROM A TOPIC ("point" from image_processing launch.launch)
And taking stimulation command (should i reach or not? is there a noise in the command? from the rat_model testbash.launch)
*/


#include <rat_model/list_pop.h>

using namespace std;
using namespace KMath::KMat;
using namespace KDeviceLists;

//default starting values
string color("blue");
float lpx=218.65; //be wary, if you change here you also have to change something down, check it out
float lpy=113;
float lpz=65;

//rat model data
double execution= 0;
float error=0;


//OBJECT POSE CALLBACK
void chatterCallback(const geometry_msgs::PointStamped& msg)
{

  ::lpx= (msg.point.x); //in mm
  ::lpy= msg.point.y; //in mm
  ::lpz= msg.point.z; //cheating here because z obtained with my caemra is not good here considering nao size
  ::color= msg.header.frame_id;
  cout <<"kinematic point received: "<< lpx << " , " << lpy << ", " << lpz<< ",color " << color << "\n";
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

//USELESS TO ERASE LATER (PART OF THE TEST BENCH)
void stimuCallback(const std_msgs::Float64& msg)
{
//First stimulation reception, when no brain model. Used to test prototype 2, the motion according to a stimulation (and synchonise files)
 //TEST UNIT
/*
 if (msg.data==0.0012){
    ::execution=0;}
 else{
    ::execution=1;
    }*/

  //cout <<"kinematic stimulation received: "<<  execution << "\n";
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

//STIMULATION (ORDER TO REACH) AND NOISE CALLBACK
void stimulationcallback(const rat_model::list_pop &msg){
    //cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXIIIIIIIMMMMTHHHHEEEEEERRRREEEEEEEE CAAAN YOUUU SEEEEEEEE MMMMEEEEEEEE?";
    ::execution=msg.popaveragerate[0];
    ::error=msg.popaveragerate[1];
    cout<<"stimulation: "<< execution <<", error: "<<error <<"\n";


}

//TO HAVE THE OBJECT POSE in end point matrix
void convert(float x,float y, float z, float *result){
    float ch=cos(x);
    float sh=sin(x);
    float ca=cos(y);
    float sa=sin(y);
    float cb=cos(z);
    float sb=sin(z);

    result[0] = ch * ca;
    result[1] = sh*sb - ch*sa*cb;
    result[2] = ch*sa*sb + sh*cb;
    result[3] = sa;
    result[4] = ca*cb;
    result[5] = -ca*sb;
    result[6] = -sh*ca;
    result[7] = sh*sa*cb + ch*sb;
    result[8] = -sh*sa*sb + ch*cb;
}



int main(int argc, char **argv){

    //ROS
    ros::init(argc, argv, "kinematic");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("jointangles", 1000);
    ros::Subscriber sub = n.subscribe("point", 1000, chatterCallback);
    ros::Subscriber sub2 = n.subscribe("/stimulation", 1000, stimuCallback);
    ros::Subscriber sub3 = n.subscribe("/brainoutstimu", 1000,stimulationcallback);
    ros::Rate loop_rate(10);

	NAOKinematics nkin;
	NAOKinematics::kmatTable output1, output2;
    string writedatapath="/home/cata/nao_ws/src/world/data/kinematic_joints/joints_decompo.txt";
	std::vector<float> joints(NUMOFJOINTS);

    string prev("");
    string prev2("");
    float prevlpx=0;
    float prevlpy=0;
    float prevlpz=0;



    unsigned int starting_point=0; //just to send a motion at the beginning, else trouble

	//while (true){
    while (ros::ok())
     {
        std_msgs::String msg;

        //position (x,y,z)
        float lrx,lry,lrz;//,rrx,rry,rrz; //rot

        ifstream write;
        write.open("/home/cata/nao_ws/src/world/data/objectangle.txt");
        if (!write) {
            cerr << "Unable to open file objectpoint.txt";
            exit(1);   // call system to stop
        }
        //left
        write >> lrx >> lry >> lrz;//rot left x,y,z
        //cout<< "lrx " <<lrx <<" lry " <<lry <<" lrz " << lrz ;
        write.close();

        if (lpx!= 218.65 and lpy!=113 and lpz!=65 ){
            starting_point=1;
        }

        //to get a random float
        boost::random::mt19937 gen;
        boost::random::uniform_real_distribution<float> dist(-abs(error),abs(error+0.0001));//-0.1001, 0.1001); //+0.0001 because it can't take 0
        /*
        If high stimulation or if we are moving to the home (starting point)
        */
        if (execution==1 or starting_point==0){
            /*
            If the new end point is different from the previous one (else we will redo the whole motion from start to end again and again)
            */

            if (not(prevlpx==lpx and prevlpy==lpy and prevlpz==lpz)or starting_point==0){
                stringstream fullstream;
                //Chose the number of sub-motion we want
                unsigned int div_number=16;

                if (abs(lpx-prevlpx)<30 and starting_point!=0){
                    div_number=6;
                    lpz=3;
                }

                //Formula to divide the end point into sub-motion
		        float lx=(lpx-prevlpx)/div_number;
                float ly=(lpy-prevlpy)/div_number;
                float lz=(40-prevlpz)/div_number;

                //For each sub-motion (from 1 to div_number, 1 because we don't want to have the starting pose as sub-motion)
                for (unsigned int div=1; div<=div_number+1; div++){

                    //If we are effectuating the first motion, we don't sub-divise it
                    if (starting_point==0){ //don't want to divide the first goal
                        div=div_number+1;
                    }
                    //cout<<"angle sent"; //check point

                    //ONLY GRABBING PART, to take off if we don't wanna grab cylinder
                    float z=prevlpz+lz*div;
                    /*
                     if (div<div_number/1.5){
                        z=z+4;//add an offset to the height, just to arrive on the cylinder from above
                    }*/

                    if (div<=div_number){
                        //the sub-motion end point, if we want we can add noise to those points by uncommenting the random.
                        output1(0,3)=(prevlpx+lx*div);//+(std::rand() % 5) ;//350;  //forward/backward
                        output1(1,3)=prevlpy+ly*div;//+std::rand() % 5;//320; //side to side
                        output1(2,3)=z;//+std::rand() % 5;//200+100; //up/down
                    }

                    //If we finished the full motion, we update the previous pose.
                    if (div==div_number){
                        prevlpx=lpx;
                        prevlpy=lpy;
                        prevlpz=lpz;
                    }
                    /*
                    if (div>div_number){
                        output1(0,3)=lpx;//+(std::rand() % 5) ;//350;  //forward/backward
                        output1(1,3)=lpy;//+std::rand() % 5;//320; //side to side
                        output1(2,3)=lpz;//+std::rand() % 5;//200+100; //up/down
                    }*/
                   if (div>div_number){
                        output1(0,3)=lpx;//+(std::rand() % 5) ;//350;  //forward/backward
                        output1(1,3)=lpy;//+std::rand() % 5;//320; //side to side
                        output1(2,3)=lpz;//+std::rand() % 5;//200+100; //up/down
                        prevlpx=lpx;
                        prevlpy=lpy;
                        prevlpz=lpz;
                    }


                    //rotation
                    float mat[9];
                    convert(lrx,lry,lrz,mat);

                    output1(0,0)=mat[0];
                    output1(0,1)=mat[1];
                    output1(0,2)=mat[2];

                    output1(1,0)=mat[3];
                    output1(1,1)=mat[4];
                    output1(1,2)=mat[5];

                    output1(2,0)=mat[6];
                    output1(2,1)=mat[7];
                    output1(2,2)=mat[8];

                    vector<vector<float> > result;

                    std::cout << "x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
                    //output1.prettyPrint();

                    joints[L_ARM+SHOULDER_PITCH]=0;
                    joints[L_ARM+SHOULDER_ROLL]=0;
                    joints[L_ARM+ELBOW_YAW]=0;
                    joints[L_ARM+ELBOW_ROLL]=0;
                    nkin.setJoints(joints);

                    //we get the joints angles.
                    result = nkin.jacobianInverseLeftHand(output1);


                    if(!result.empty()){
                        stringstream littlestream,longstream ;
                        ofstream read;
                        read.open(writedatapath, std::ios::app); //In this txt we write all the data of a stimulation
                        //we write all the info of the ball and the points of the sub motion
                        littlestream<<"\n color:"<<color<<" x:"<<output1(0,3)<<" y:"<<output1(1,3)<<" z:"<<output1(2,3)<< " error:"<< error <<  " angles: ";
                        string s1 = littlestream.str();

                        //We don't want to write consequitevely the same data
                        if (prev.compare(s1)!=0){
                            read << s1 << ",";
                            prev=s1;
                        }//prev.compare

                        for(unsigned int j=0; j<result[0].size(); j++){
                            stringstream stream;

                            /*if (error>0){
                                float merror=dist(gen);
                            }*/

                            float merror=dist(gen);
                            cout<<"NOISE :"<< merror<<"\n";
                            if (div<div_number){
                                stream << fixed << setprecision(4) << result[0][j]+merror;
                            }
                            else{
                                stream << fixed << setprecision(4) << result[0][j];
                            }
                            //cout << "angle" << j << " = " << result[0][j]/div << " ";

                            fullstream<<stream.str()<<","; //to send in the topic
                            longstream<<stream.str()<<","; //to write down in the txt file
                        }//j

                        string s2 = longstream.str();
                        if (prev2.compare(s2)!=0){
                            read << "  "<< div << " :  ";
                            read << s2 << ",";
                            prev2=s2;
                        }//prev compare

                        read.close();
                    }//if not empty
                 msg.data = fullstream.str(); //we add the new joints position to the list
                 }//for div
                chatter_pub.publish(msg); //and send all the joints position at once
                ros::Duration(0.09).sleep(); // sleep in second to be sure

                }//if not = prev
        }//end if execution

    //IF no stimulation, no data
    else{
        stringstream fullstream;
        //cout <<"POSITION NOT REACHABLE";
        fullstream<<"POSITION NOT REACHABLE";
        msg.data = fullstream.str();
        chatter_pub.publish(msg);

    }
    ros::spinOnce();
    loop_rate.sleep();
  }

	return 0;
}

