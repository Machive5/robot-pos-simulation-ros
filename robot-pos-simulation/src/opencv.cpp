#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>

using namespace ros;
using namespace std;
using namespace cv;

float distance(float a, float b){
    return sqrt(a*a + b*b);
}

void setup(NodeHandle nh){
    ServiceClient client = nh.serviceClient<turtlesim::Kill>("/kill",1);
    turtlesim::Kill kill;
    kill.request.name = "turtle1";
    client.call(kill);
    spinOnce();

    client = nh.serviceClient<turtlesim::Spawn>("/spawn",1);
    turtlesim::Spawn spwn;
    spwn.request.name = "turtle1";
    spwn.request.x = 1;
    spwn.request.y = 1;
    client.call(spwn);
    spinOnce();
}

int main(int argc, char** argv){

    //node init
    init(argc, argv, "opencv");
    NodeHandle nh;

    //setup turtle sim
    setup(nh);

    ServiceClient client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute",1);


    //opencv code
    VideoCapture cap("/home/abrar/Documents/robotik/P3/robot-pos-simulator/src/robot-pos-simulation/src/video/Video.mp4");
    Mat frame;
    Mat resized;
    Mat HSV;
    Mat thres;

    vector<vector<Point>> contours;

    int h[2] = {100,255};
    int s[2] = {155,255};
    int v[2] = {200,255};

    Point2f center;
    Moments m;
    float radius = 0;
    vector<double> pos0;
    float ofside = 1;

    while (ok())
    {
        cap >> frame;

        cvtColor(frame, HSV, COLOR_RGB2HSV);
        inRange(HSV,Scalar(h[0], s[0], v[0]),Scalar(h[1], s[1], v[1]),thres);

        findContours(thres, contours, RETR_TREE, CHAIN_APPROX_NONE);

        if (contours.size()>0){
            minEnclosingCircle(contours[0], center, radius);
            m = moments(contours[0]);
            circle(frame, center, cvRound(radius), Scalar(0,255,0), 2);
            circle(frame, Point((frame.cols/2)-15,(frame.rows/2)-40), cvRound(5), Scalar(255,0,0), FILLED);

            if (pos0.size() == 0){
                pos0.push_back(m.m10/m.m00);
                pos0.push_back(m.m01/m.m00);

                //spawn turtle2
                ServiceClient turtle2  = nh.serviceClient<turtlesim::Spawn>("/spawn",1);
                turtlesim::Spawn spwn;
                spwn.request.name = "turtle2";
                spwn.request.x = ofside+(pos0[0]-(frame.cols/2))/100;
                spwn.request.y = ofside+((frame.rows/2)-pos0[1])/100;
                turtle2.call(spwn);
                spinOnce();
            }

            double x = (pos0[0]-(m.m10/m.m00));
            double y = ((m.m01/m.m00)-pos0[1]);
            
            if (m.m00 != 0){

                Point tKoor(30,30);
                putText(frame, "Posisi: (" + to_string(x/10) + "cm, " + to_string(y/10)+"cm)", tKoor, QT_FONT_NORMAL, 1, Scalar(0,0,255), 1);

                turtlesim::TeleportAbsolute msg;
                msg.request.x = (x/100)+ofside;
                msg.request.y = (y/100)+ofside;
                
                client.call(msg);
                spinOnce();
                
            }
        }

        resize(frame,resized,Size(frame.cols/2, frame.rows/2));
        imshow("video", resized);

        if(waitKey(10) == 'q'){
            break;
        }
    }
    

    return 0;
}