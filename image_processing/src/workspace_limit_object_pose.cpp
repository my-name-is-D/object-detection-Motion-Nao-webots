#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

#include <geometry_msgs/PointStamped.h>
#include<string>
#include<iostream>
#include <math.h>

#define focalLenght 605
//#define dZ0 450 //here Z is fixed.
#define alfa 39.7 // angle of the camera center from the z position real
#define h 44.2 //height from camera to nao hand at pose 0 in mm
#define d 147.84 // dist on x from camera and nao hand at pose 0
#define PI 3.14159265
void objectsDetectedCallback(const std_msgs::Float32MultiArray& msg){
    ros::NodeHandle nh;
    ros::Publisher position_pub_=nh.advertise<geometry_msgs::PointStamped>("point", 1);

    //find_object_2d::PointObjects p_objects;
    //find_object_2d::Point_id objeto;

    geometry_msgs::PointStamped p_objects;
    //p_objects.objeto = std::vector<find_object_2d::Point_id>(msg.data.size()/12);
    //CHECK
    if (msg.data.size() >1 ) {
        std::cout<<"data.size" <<std::endl;
        std::cout<<msg.data.size() <<std::endl;
    }
    for(unsigned int i=0; i<msg.data.size(); i+=12)
    {
        // get data
        int id = (int)msg.data[i];

        //WORKSPACE LENGHT
        float Dist_arm=55.95+105.0+57.75; //219 offsethandx, upper and lower arm size
        float OffsetLY= 98.0+15.0; //113 offset shoulder and elbow
        float OffsetLZ=75.0+12.31; //112 offset shoulder and hand (see kinematicdefine.h)
        //WORKSPACE RESTRICTION (PARTIAL)
        float second_Dist_arm= Dist_arm+ OffsetLY;
        float third_Dist_arm=Dist_arm+OffsetLZ;

        float objectWidth = msg.data[i+1];
        float objectHeight =msg.data[i+2];
        //int camera_center = 640; // left 0, right 1280 -resolution 720p


        // Find corners OpenCV
        cv::Mat cvHomography(3, 3, CV_32F);
        std::vector<cv::Point2f> inPts, outPts;
        //extract coef of homography matrix
        cvHomography.at<float>(0, 0) = msg.data[i+3];
        cvHomography.at<float>(1, 0) = msg.data[i+4];
        cvHomography.at<float>(2, 0) = msg.data[i+5];
        cvHomography.at<float>(0, 1) = msg.data[i+6];
        cvHomography.at<float>(1, 1) = msg.data[i+7];
        cvHomography.at<float>(2, 1) = msg.data[i+8];
        cvHomography.at<float>(0, 2) = msg.data[i+9];
        cvHomography.at<float>(1, 2) = msg.data[i+10];
        cvHomography.at<float>(2, 2) = msg.data[i+11];

        //defining corners (input plane)
        inPts.push_back(cv::Point2f(0, 0)); //top left
        inPts.push_back(cv::Point2f(objectWidth, 0)); //top right
        inPts.push_back(cv::Point2f(0, objectHeight)); //bottom left
        inPts.push_back(cv::Point2f(objectWidth, objectHeight)); //bottom right
        //Calcultaing perspective (straigthen corners )
        cv::perspectiveTransform(inPts, outPts, cvHomography);

        /*
        QTransform qtHomography(msg.data[i+3], msg.data[i+4], msg.data[i+5],
                                msg.data[i+6], msg.data[i+7], msg.data[i+8],
                                msg.data[i+9], msg.data[i+10], msg.data[i+11]);

        QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
        QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
        QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
        QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));
       */
        geometry_msgs::Point punto;
    /*
	float widthTop = sqrt(pow(qtTopRight.x() - qtTopLeft.x(),2) + pow(qtTopRight.y() - qtTopLeft.y(),2));
	float widthBottom = sqrt(pow(qtBottomRight.x() - qtBottomLeft.x(),2) + pow(qtBottomRight.y() - qtBottomLeft.y(),2));
	float heightLeft = sqrt(pow(qtBottomLeft.x() - qtTopLeft.x(),2) + pow(qtBottomLeft.y() - qtTopLeft.y(),2));
	float heightRight = sqrt(pow(qtBottomRight.x() - qtTopRight.x(),2) + pow(qtBottomRight.y() - qtTopRight.y(),2));
	*/
        float widthTop = sqrt(pow(outPts.at(1).x - outPts.at(0).x,2) + pow(outPts.at(1).y - outPts.at(0).y,2));
	    float widthBottom = sqrt(pow(outPts.at(3).x - outPts.at(2).x,2) + pow(outPts.at(3).y - outPts.at(2).y,2));
	    float heightLeft = sqrt(pow(outPts.at(2).x - outPts.at(0).x,2) + pow(outPts.at(2).y - outPts.at(0).y,2));
	    float heightRight = sqrt(pow(outPts.at(3).x - outPts.at(1).x,2) + pow(outPts.at(3).y - outPts.at(1).y,2));

        //X centre point
        //float x_pos = (int)(outPts.at(0).x + outPts.at(1).x + outPts.at(2).x +outPts.at(3).x) /4;
        float dArea_0 = ((widthTop + widthBottom)/2) * ((heightLeft + heightRight)/2);
        std::cout<<"area in px" <<std::endl;
        std::cout<<((outPts.at(0).y + outPts.at(1).y + outPts.at(2).y + outPts.at(3).y) /4)-360 <<std::endl;
        float realarea;
        switch(id){
        case 2:
            realarea= 8800;
            break;
        case 3:
            realarea= 3268;
            break;
        default:
            realarea=0;

        }
        //camera distance to the object //X FRONT ROBOT
        float dX= focalLenght* realarea/dArea_0;

        //degree of camera: 65 (let's say) ,my webcam resolution: 720p (height resolution)
        //we take the center of the square.
        float anglex = -1*((65*(((outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) /4)/720)) - 28.5); //find the angle of view of the object (in x)

        //We determinate the pose of the object according to Z
        //0.2645833333 mm=1px
        float dZ = -1*0.2645833333*((((outPts.at(0).y + outPts.at(1).y + outPts.at(2).y + outPts.at(3).y) /4))- 360); // calculate the z position according to the camera //Z UP DOWN ROBOT
        float dY = dX*std::sin(0 + anglex*PI/180); // calculate the y position according to the camera //Y LEFT RIGHT ROBOT //if the robot moves then it won't be 0 -i keep it as a reminder
        //float dZ_0 = dZ0 + (dArea_0/10);

        punto.x = dX;// then there is the distance from camera to hand at pose 0 to consider. Here it won't be.
        punto.y = dY;
        punto.z = dZ;


        /*
        float dY_0 = (((480/2) - (((outPts.at(0).y + outPts.at(1).y)/2) + ((heightLeft + heightRight)/4)))*dZ0)/585;

        float beta_0 = atan2(dY_0,dZ0);

        objectHeight = objectHeight/cos((alfa*PI)/180);

        float height = ((heightLeft + heightRight)/2)/cos(((alfa*PI)/180)-beta_0);

        float dArea = (objectHeight*objectWidth) - (((widthTop + widthBottom)/2) * height);

        float dZ = dZ0 + (dArea/38);

        float dX = (((640/2) - (((outPts.at(0).x + outPts.at(2).x)/2) + ((widthTop + widthBottom)/4)))*dZ)/585;

        float dY = (((480/2) - (((outPts.at(0).y + outPts.at(1).y)/2) + ((heightLeft + heightRight)/4)))*dZ)/585;

        float beta = atan2(dY,dZ);
        */

        /*
        punto.x = dX;
        punto.y = h-((dZ/cos(beta))*sin(((alfa*PI)/180)-beta));
        punto.z = ((dZ/cos(beta))*cos(((alfa*PI)/180)-beta))-d;
        */
        //Validate detection
        int paralelepipedo;
        std::cout << "width" <<std::endl;
        std::cout << widthTop <<std::endl;
        std::cout << "height" <<std::endl;
        std::cout << heightLeft <<std::endl;

        //if square around object too deformed, we don't keep it
        if (abs(widthTop - widthBottom) < 20 && abs(heightLeft - heightRight) < 15)
        {
            paralelepipedo = 1;

            if (!(sqrt(punto.x*punto.x+(punto.y-OffsetLY)*(punto.y-OffsetLY))<= second_Dist_arm && 0<= acos(punto.x/Dist_arm) &&
             acos(punto.x/Dist_arm)<=acos(0/Dist_arm) &&
             asin((-67-OffsetLY)/Dist_arm)<= asin((punto.y-OffsetLY)/Dist_arm) &&
             asin((punto.y-OffsetLY)/Dist_arm)<=asin((331-OffsetLY)/Dist_arm))){
                //0 correspond to max point when x=Dist_arm)
                //-67 is the min y pose and 331 is the max (with offset)
                paralelepipedo=0;
                std::cout<< "Not in Nao's workspace ";
            }//y/D x/D and DIST X Y

            if(!(sqrt(punto.x*punto.x+(punto.z-OffsetLZ)*(punto.z-OffsetLZ))<= third_Dist_arm &&
            acos((305.9-OffsetLZ)/Dist_arm)<= acos((punto.z-OffsetLZ)/Dist_arm) &&
            acos((punto.z-OffsetLZ)/Dist_arm)<=acos((-80-OffsetLZ)/Dist_arm) &&
            asin(0/Dist_arm)<=asin(punto.x/Dist_arm)))
            {
            paralelepipedo=0;
        //We don't let x go behind its back, 305.9 and -80 are the max and min z with offset.
            std::cout<< "Not in Nao's workspace";

            }

		}
		else
		{
			paralelepipedo = 0;
		}

        if (paralelepipedo == 1)
        {


			//objeto.punto = punto;
			//objeto.id = id;
            p_objects.header.stamp = ros::Time::now();
            p_objects.header.frame_id =  std::to_string(id);
            p_objects.point.x= punto.x;
            p_objects.point.y= punto.y;
            p_objects.point.z= punto.z;


			//p_objects.objeto[i/12] = objeto;

            std::cout << "P_objects" <<std::endl;
            std::cout << p_objects <<std::endl;

            position_pub_.publish(p_objects);
	}

    }



}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "objects_detected");

    ros::NodeHandle nh;
    ros::Subscriber subs = nh.subscribe("objects", 1, objectsDetectedCallback);

    ros::Publisher position_pub_=nh.advertise<geometry_msgs::PointStamped>("point", 1);//nh.advertise<find_object_2d::PointObjects>("point", 1);

    ros::spin();

    return 0;
}
