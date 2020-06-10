#include <iostream>
#include <vector>
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
#include "KinematicsDefines.h"
using namespace std;
using namespace KMath::KMat;
using namespace KDeviceLists;

//MAIN TAKING OBJECT POSE FROM A FILE
//taken from noobswestand github : NAaoPythonIK and modified for ROS/my situation.
//CAN BE ADAPTED FOR LEFT AND RIGHT ARM

float clamp(float x, float min, float max){
    if (x<min){x=min;}
    if (x>max){x=max;}
    return x;
}

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
    ros::init(argc, argv, "kinematic");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("jointangles", 1000);
    ros::Rate loop_rate(10);
	NAOKinematics nkin;
	NAOKinematics::kmatTable output1, output2;

	std::vector<float> joints(NUMOFJOINTS);
	//double pi = KMath::KMat::transformations::PI;
    std_msgs::String msg;
	//while (true){
    while (ros::ok())
     {
        //position (x,y,z)
        float lpx,lpy,lpz; //rpx,rpy,rpz; //pos
        float lrx,lry,lrz;//,rrx,rry,rrz; //rot
        float Dist_arm=HandOffsetX+UpperArmLength+LowerArmLength; //219
        float OffsetLY= ShoulderOffsetY+ElbowOffsetY; //113
        float OffsetLZ=ShoulderOffsetZ+HandOffsetZ; //112
        //float lrx,lry,lrz,rrx,rry,rrz; //rot
        bool execution= true; //if position not ok execution=false

        lrx=0;//lrx+3.1415;//lrx;
        lry=0;//lry;//lrz;
        lrz=0;//temp;

        ifstream write;
        write.open("/home/cata/nao_ws/src/world/data/objectpoint.txt");
        if (!write) {
            cerr << "Unable to open file objectpoint.txt";
            exit(1);   // call system to stop
        }
        //left
        write >> lpx >> lpy >> lpz;//pos x,y,z
        write >> lrx >> lry >> lrz;//rot x,y,z
        //right
        //write >> rpx >> rpy >> rpz;//pos x,y,z
        //write >> rrx >> rry >> rrz;//rot x,y,z

        write.close();
        //WORKSPACE RESTRICTION (PARTIAL)
        float second_Dist_arm= Dist_arm+ OffsetLY;
        float third_Dist_arm=Dist_arm+OffsetLZ;
//To consider the Offset of y (on y, the dist max is 331.7 while on x it's 218)
//on z the dist max is 112+ 218 (from the origin, not from shoulder)


        if (lpy<0 || !(sqrt(lpx*lpx+(lpy-OffsetLY)*(lpy-OffsetLY))<= second_Dist_arm && 0<= acos(lpx/Dist_arm) &&
         acos(lpx/Dist_arm)<=acos(0/Dist_arm) &&
         asin((-67-OffsetLY)/Dist_arm)<= asin((lpy-OffsetLY)/Dist_arm) &&
         asin((lpy-OffsetLY)/Dist_arm)<=asin((331-OffsetLY)/Dist_arm))){
            //0 correspond to max point when x=Dist_arm)
            //-67 is the min y pose and 331 is the max (with offset)
            execution=false;
            cout<< "IN THE IF Y ALLRIGHT ";
        }//y/D x/D and DIST X Y
        if(!(sqrt(lpx*lpx+(lpz-OffsetLZ)*(lpz-OffsetLZ))<= third_Dist_arm &&
        acos((305.9-OffsetLZ)/Dist_arm)<= acos((lpz-OffsetLZ)/Dist_arm) &&
        acos((lpz-OffsetLZ)/Dist_arm)<=acos((-80-OffsetLZ)/Dist_arm) &&
        asin(0/Dist_arm)<=asin(lpx/Dist_arm)))
        {
        //We don't let x go behind its back, 305.9 and -80 are the max and min z with offset.
            cout<< "IN THE IF Z ALLRIGHT ";
            execution=false;
        }

        if (execution){

            //MATRIX COMPOSITION
            output1(0,3)=lpx;//350;  //forward/backward
            output1(1,3)=lpy;//320; //side to side
            output1(2,3)=lpz;//200+100; //up/down
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
            /**
            * This is a message object. You stuff it with data, and then publish it.
            */

            std::cout << "x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
            //output1.prettyPrint();

            joints[L_ARM+SHOULDER_PITCH]=0;
            joints[L_ARM+SHOULDER_ROLL]=0;
            joints[L_ARM+ELBOW_YAW]=0;
            joints[L_ARM+ELBOW_ROLL]=0;
            nkin.setJoints(joints);
            result = nkin.jacobianInverseLeftHand(output1);
            if(!result.empty()){
                ofstream read;
                read.open("read.txt");
                stringstream fullstream;
                for(unsigned int j=0; j<result[0].size(); j++){
                    stringstream littlestream, stream;
                    cout << "angle" << j << " = " << result[0][j] << " ";
                    stream << fixed << setprecision(4) << result[0][j];

                    fullstream<<stream.str()<<",";

                    string s = stream.str();
                    read << s << ",";
                    //cout << s << " , ";
                }
                read.close();
                msg.data = fullstream.str();
            }
        }//end if execution

        /*
        result = nkin.inverseLeftHand(output1);
	    //cout<<"test2"<<endl;
	    if(!result.empty()){
		    cout << "--Solution exists 1" << endl;
		    for(unsigned int j=0; j<result[0].size(); j++){
			    cout << "angle" << j << " = " << result[0][j] << " ";
		    }
	    }
	    else{
	        cout<<"no solution"<<endl;
	    }
		cout << endl;
		*/
	else{
        stringstream fullstream;
        cout <<"POSITION NOT REACHABLE";
        fullstream<<"POSITION NOT REACHABLE";
        msg.data = fullstream.str();

    }
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

	return 0;
}

