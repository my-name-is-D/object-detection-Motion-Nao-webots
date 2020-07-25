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

float rry=-113;

//rat model data
double execution= 0;
float error=0;


//OBJECT POSE CALLBACK
void PointCallback(const geometry_msgs::PointStamped& msg)
{

  ::lpx= (msg.point.x); //in mm
  ::rry=::lpy= msg.point.y; //in mm
  ::lpz= msg.point.z; //cheating here because z obtained with my caemra is not good here considering nao size
  ::color= msg.header.frame_id;
  cout <<"kinematic point received: "<< lpx << " , " << lpy << ", " << lpz<< ",color " << color << "\n";
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
    ros::Publisher left_pub = n.advertise<std_msgs::String>("jointangles", 1000);
    ros::Publisher right_pub = n.advertise<std_msgs::String>("rightjointangles", 1000);
    ros::Subscriber sub = n.subscribe("point", 1000, PointCallback);
    ros::Subscriber sub3 = n.subscribe("/brainoutstimu", 1000,stimulationcallback);
    ros::Rate loop_rate(10);

	NAOKinematics nkin, nkinright;
	NAOKinematics::kmatTable output1, output2;
    string writedatapath="/home/cata/nao_ws/src/world/data/kinematic_joints/2hands_joints_decompo.txt";
	std::vector<float> joints(NUMOFJOINTS);

    string prevl("");
    string prevl2("");
    string prevr("");
    string prevr2("");

    float prevlpx=0;
    float prevlpy=0;
    float prevlpz=0;
    float prevrpy=0;



    unsigned int starting_point=0; //just to send a motion at the beginning, else trouble

	//while (true){
    while (ros::ok())
     {
        std_msgs::String msgleft;
        std_msgs::String msgright;

        //ROTATIONS
        float lrx,lry,lrz;//,rrx,rry,rrz; //rot
        lrx=0;//empirical observation
        lry=0;
        lrz=0;
        float angrx=0;
        float angry=0;
        float angrz=0;

        //We left the home position
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
                stringstream leftfullstream, rightfullstream;
                //Chose the number of sub-motion we want
                unsigned int div_number=16;

                //Formula to divide the end point into sub-motion
		        float lx=(lpx-prevlpx)/div_number;
                float ly=(lpy-prevlpy)/div_number;
                float lz=(lpz-prevlpz)/div_number;
                float ry=(rry-prevrpy)/div_number; //It's different from ly by symmetry

                //For each sub-motion (from 1 to div_number, 1 because we don't want to have the starting pose as sub-motion)
                for (unsigned int div=1; div<=div_number; div++){

                    //If we are effectuating the first motion, we don't sub-divise it
                    if (starting_point==0){ //don't want to divide the first goal
                        div=div_number;
                    }
                    //cout<<"angle sent"; //check point

                    //the sub-motion end point, if we want we can add noise to those points by uncommenting the random.
                    output2(0,3)=output1(0,3)=(prevlpx+lx*div);//+(std::rand() % 5) ;//350;  //forward/backward
                    output1(1,3)=prevlpy+ly*div;//+std::rand() % 5;//320; //side to side

                    output2(1,3)=prevrpy+ry*div;

                    output2(2,3)=output1(2,3)=prevlpz+lz*div;//+std::rand() % 5;//200+100; //up/down

                    //If we finished the full motion, we update the previous pose.
                    ifstream write;
                    write.open("/home/cata/nao_ws/src/world/data/objectangle.txt");
                    if (!write) {
                        cerr << "Unable to open file objectpoint.txt";
                        exit(1);   // call system to stop
                    }
                    //left
                    write >> lrx >> lry >> lrz;//rot left x,y,z
                    //right
                    write >> angrx >> angry >> angrz;//rot right x,y,z

                    write.close();

                    /*
                    lrx=0;//empirical observation
                    lry=30;//((float)div_number-(float)div+1);
                    lrz=10;///((float)div_number-(float)div+1);
                    angry=-5;///((float)div_number-(float)div+1);
                    angrz=5;///((float)div_number-(float)div+1);
                    angrz=-angrz;
                    */
                    cout << " div number - div " << lrx;
                    cout << " div number - div " << lry;
                    cout << " div number - div " << lrz;

                    cout << " div number - div " << angry;
                    cout << " div number - div " << angrz;


                    if (div==div_number){
                        prevlpx=lpx;
                        prevlpy=lpy;
                        prevlpz=lpz;
                        prevrpy=rry;
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

                    float mat1[9];
                    convert(angrx,angry,angrz,mat1);


                    output2(0,0)=mat1[0];
                    output2(0,1)=mat1[1];
                    output2(0,2)=mat1[2];

                    output2(1,0)=mat1[3];
                    output2(1,1)=mat1[4];
                    output2(1,2)=mat1[5];

                    output2(2,0)=mat1[6];
                    output2(2,1)=mat1[7];
                    output2(2,2)=mat1[8];

                    vector<vector<float> > resultl;
                    vector<vector<float> > resultr;


                    std::cout << "LEFT x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
                    std::cout << "RIGHT x = " << output2(0,3) << " y = " << output2(1,3) << " z = " << output2(2,3) <<  std::endl;

                    //output1.prettyPrint();

                    joints[L_ARM+SHOULDER_PITCH]=0;
                    joints[L_ARM+SHOULDER_ROLL]=0;
                    joints[L_ARM+ELBOW_YAW]=0;
                    joints[L_ARM+ELBOW_ROLL]=0;
                    nkin.setJoints(joints);

                    //we get the joints angles.
                    resultl = nkin.jacobianInverseLeftHand(output1);

                    joints[R_ARM+SHOULDER_PITCH]=0;
                    joints[R_ARM+SHOULDER_ROLL]=0;
                    joints[R_ARM+ELBOW_YAW]=0;
                    joints[R_ARM+ELBOW_ROLL]=0;
                    nkinright.setJoints(joints);
                    resultr = nkinright.jacobianInverseRightHand(output2);


                    if(!resultl.empty() and !resultr.empty()){
    //-------------------------------------------LEFT ARM--------------------------------------------------//
                        stringstream littlestream,longstream;
                        ofstream read;
                        read.open(writedatapath, std::ios::app); //In this txt we write all the data of a stimulation
                        //we write all the info of the ball and the points of the sub motion
                        littlestream<<"\n LEFT HAND, color:"<<color<<" x:"<<output1(0,3)<<" y:"<<output1(1,3)<<" z:"<<output1(2,3)<< " error:"<< error <<  " angles: ";

                        string sl1 = littlestream.str();

                        //We don't want to write consequitevely the same data
                        if (prevl.compare(sl1)!=0){
                            read << sl1 << ",";
                            prevl=sl1;
                        }//prev.compare LEFT

                        for(unsigned int j=0; j<resultl[0].size(); j++){
                            stringstream stream;

                            /*if (error>0){
                                float merror=dist(gen);
                            }*/
                            float merror=dist(gen);
                            cout<<"LEFT NOISE :"<< merror<<"\n";
                            //cout << "angle" << j << " = " << resultl[0][j]/div << " ";
                            stream << fixed << setprecision(4) << resultl[0][j]+merror;
                            leftfullstream<<stream.str()<<","; //to send in the topic
                            longstream<<stream.str()<<","; //to write down in the txt file
                        }//j

                        string sl2 = longstream.str();
                        if (prevl2.compare(sl2)!=0){
                            read << "  "<< div << " :  ";
                            read << sl2 << ",";
                            prevl2=sl2;
                        }//prev compare LEFT

    //-------------------------------------------RIGHT ARM--------------------------------------------------//
                        stringstream rightlittlestream,rightlongstream;

                        //we write all the info of the ball and the points of the sub motion
                        rightlittlestream<<"\n RIGHT HAND, color:"<<color<<" x:"<<output2(0,3)<<" y:"<<output2(1,3)<<" z:"<<output2(2,3)<< " error:"<< error <<  " angles: ";

                        string sr1 = rightlittlestream.str();

                        //We don't want to write consequitevely the same data
                        if (prevr.compare(sr1)!=0){
                            read << sr1 << ",";
                            prevr=sr1;
                        }//prev.compare LEFT

                        for(unsigned int j=0; j<resultr[0].size(); j++){
                            stringstream stream;

                            /*if (error>0){
                                float merror=dist(gen);
                            }*/
                            float merror=dist(gen);
                            //cout<<"RIGHT NOISE :"<< merror<<"\n";
                            //cout << "angle" << j << " = " << resultl[0][j]/div << " ";
                            stream << fixed << setprecision(4) << resultr[0][j]+merror;
                            rightfullstream<<stream.str()<<","; //to send in the topic
                            rightlongstream<<stream.str()<<","; //to write down in the txt file
                        }//j

                        string sr2 = rightlongstream.str();
                        if (prevr2.compare(sr2)!=0){
                            read << "  "<< div << " :  ";
                            read << sr2 << ",";
                            prevr2=sr2;
                        }//prev compare LEFT

                        read.close();
                    }//if not empty
                 msgleft.data = leftfullstream.str(); //we add the new joints position to the list
                 msgright.data = rightfullstream.str(); //we add the new joints position to the list
                 }//for div
                left_pub.publish(msgleft); //and send all the joints position at once
                right_pub.publish(msgright); //and send all the joints position at once
                ros::Duration(0.05).sleep(); // sleep in second to be sure

                }//if not = prev
        }//end if execution

    //IF no stimulation, no data
    else{
        stringstream fullstream;
        //cout <<"POSITION NOT REACHABLE";
        fullstream<<"POSITION NOT REACHABLE";
        msgleft.data = fullstream.str();
        right_pub.publish(msgleft);
        left_pub.publish(msgleft);

    }
    ros::spinOnce();
    loop_rate.sleep();
  }

	return 0;
}

