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

	//while (true){
    while (ros::ok())
     {
        //position (x,y,z)
        float lpx,lpy,lpz; //rpx,rpy,rpz; //pos
        float lrx,lry,lrz;//,rrx,rry,rrz; //rot
        //float lrx,lry,lrz,rrx,rry,rrz; //rot
/*
        lpx=180;
        lpy=30;
        lpz=0.0;
        */


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

        /*
        //min/max left
        lpz=clamp(lpz,-.95,.95);//up down
        lpy=clamp(lpy,-.05,.85);//left right
        lpx=clamp(lpx,0,.67);//foward back

        //convert space
        //float temp=lry;
        lrx=0;//lrx+3.1415;//lrx;
        lry=0;//lry;//lrz;
        lrz=0;//temp;

        output1(0,3)=lpx*304.8;//350;  //forward/backward
        output1(1,3)=lpy*360;//320; //side to side
        output1(2,3)=lpz*204.8+100;//200+100; //up/down
        */

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
        std_msgs::String msg;


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

/*
        //right
        rrx=0;//rrx+3.1415;//lrx;
        rry=0;//rry;//lrz;
        rrz=0;//temp;


        //min/max right
        rpz=clamp(rpz,-.95,.95);//up down
        rpy=clamp(rpy,-.85,.05);//left right
        rpx=clamp(rpx,0,.67);//foward back


        output1(0,3)=rpx*304.8;  //forward/backward
        //output1(1,3)=rpy*304.8; //side to side
        output1(1,3)=rpy*360;//
        output1(2,3)=rpz*204.8+100; //up/down

        //rotation
        //float mat[9];

        convert(rrx,rry,rrz,mat);

        output1(0,0)=mat[0];
        output1(0,1)=mat[1];
        output1(0,2)=mat[2];

        output1(1,0)=mat[3];
        output1(1,1)=mat[4];
        output1(1,2)=mat[5];

        output1(2,0)=mat[6];
        output1(2,1)=mat[7];
        output1(2,2)=mat[8];

        //vector<vector<float> > result;
        std::cout << "x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
        //output1.prettyPrint();
        joints[R_ARM+SHOULDER_PITCH]=0;
        joints[R_ARM+SHOULDER_ROLL]=0;
        joints[R_ARM+ELBOW_YAW]=0;
        joints[R_ARM+ELBOW_ROLL]=0;
        nkin.setJoints(joints);
        result = nkin.jacobianInverseRightHand(output1);
        if(!result.empty()){
            std::ofstream read;
            read.open("read.txt", std::ios_base::app);
            for(unsigned int j=0; j<result[0].size(); j++){
                //cout << "angle" << j << " = " << result[0][j] << " ";
                stringstream stream;
                stream << fixed << setprecision(4) << result[0][j];
                string s = stream.str();
                read << s << "\n";
                //cout << s << " , ";
            }
            read.close();
        }
        */

	//}


    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

	return 0;
}

