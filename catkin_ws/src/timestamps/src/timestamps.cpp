#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>
#include "timestamps/conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <bits/stringfwd.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <Eigen/Eigen>
#include<vector>
#include<map>
#include <math.h>
#define M_PI       3.14159265358979323846

using namespace std;  
std::map<double, geometry_msgs::PoseStamped> hector_timetable;
std::map<double, geometry_msgs::PoseStamped> gps_timetable;

std::string hector_topic	= "/slam_out_pose";
std::string gps_topic		= "/fix";
int startIndex=0;
Eigen::Affine3d startTransform = Eigen::Affine3d::Identity();
// Eigen::Affine3d ecef2ned;
double ecef2ned1[3],ecef2nedfirst[3],Rota[9];
double pos_ned[3];
double Rotainti[9];
double yaw=255*M_PI/180;
double pos_rota[3];
geometry_msgs::PoseStamped pose_min0;
void callback_gps(const sensor_msgs::NavSatFixConstPtr &fix) {

        if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
         ROS_INFO("No fix.");
        return;
         }

       if (fix->header.stamp == ros::Time(0)) {
        return;
        }
        geometry_msgs::PoseStamped pose;
        pose.header = fix->header;

        double lat, lon , alt;
        lat=(fix->latitude)*M_PI/180;
        lon=(fix->longitude)*M_PI/180;
        alt=(fix->altitude);

        double Rea=6378137;
        double f=1/298.257223563;
        double Reb=Rea*(1-f);
        double e=sqrt(Rea*Rea - Reb*Reb)/Rea;
        double Ne=Rea/sqrt(1 - e*e*sin(lat)*sin(lat));

        double xe = (Ne + alt) * cos(lat) * cos(lon);
        double ye = (Ne + alt) * cos(lat) * sin(lon);
        double ze = (Ne*(1 -e*e) + alt) * sin(lat);

        Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(xe, ye, ze)));

        // Transform ECEF frame to NED frame
		// compute rotation only for the first GPS point, to define the coordinate system
		/*
			ecef2ned;	// global variables
			ned2ros;
			rotation;

			if startIndex<0
				ecef2ned=affine3d(lat,lon)
				ned2ros =affine3d...
				rotation = affine3d...
			
			transform=translation*ecef2ned*ned2rosi
				
				
		*/
		

        // Transform NED frame to ROS frame (x front, y left, z up)
        Eigen::Affine3d ned2ros = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0)));

        // Local rotation from inclination sensors
        //Eigen::Affine3d rotation = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0))) *
          //      Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
          //      Eigen::Affine3d(Eigen::AngleAxisd(1.04, Eigen::Vector3d(0, 0, 1)));

        

        // Use relative pose to ROS frame of pose at startIndex
        if (startIndex<2) {

           // ecef2ned = Eigen::Affine3d(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0, 1, 0))) *
           // Eigen::Affine3d(Eigen::AngleAxisd(lon, Eigen::Vector3d(1, 0, 0))) *
           // Eigen::Affine3d(Eigen::AngleAxisd(-lat, Eigen::Vector3d(0, 1, 0)));
           Rota[0]=(-sin(lat))*cos(lon);
           Rota[1]=-sin(lat)*sin(lon);
           Rota[2]=cos(lat);
           Rota[3]=-sin(lon);
           Rota[4]=cos(lon);
           Rota[5]=0;
           Rota[6]=-cos(lat)*cos(lon);
           Rota[7]=-cos(lat)*sin(lon);
           Rota[8]=-sin(lat);
            ecef2nedfirst[0]=Rota[0]*xe+Rota[1]*ye+Rota[2]*ze;
            ecef2nedfirst[1]=Rota[3]*xe+Rota[4]*ye;
            ecef2nedfirst[2]=Rota[6]*xe+Rota[7]*ye-Rota[8]*ze; 
            //startTransform = ((ecef2ned1));

            startIndex++;
        }

        ecef2ned1[0]=Rota[0]*xe+Rota[1]*ye+Rota[2]*ze;
        ecef2ned1[1]=Rota[3]*xe+Rota[4]*ye;
        ecef2ned1[2]=Rota[6]*xe+Rota[7]*ye-Rota[8]*ze; 
        //Eigen::Affine3d transform = ((translation * ecef2ned));
        
        pos_ned[0]=ecef2ned1[0]-ecef2nedfirst[0];
        pos_ned[1]=ecef2ned1[1]-ecef2nedfirst[1];
        pos_ned[2]=ecef2ned1[2]-ecef2nedfirst[2];

        Rotainti[0]=cos(yaw);
        Rotainti[1]=sin(yaw);
        Rotainti[2]=0;
        Rotainti[3]=-sin(yaw);
        Rotainti[4]=cos(yaw);
        Rotainti[5]=0;
        Rotainti[6]=0;
        Rotainti[7]=0; 
        Rotainti[7]=1.0; 
       // Eigen::Affine3d poseg = startTransform.inverse() * transform;
        // TODO: place your coordinate conversion here
       // LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
	//Eigen::Affine3d poseg = ((translation * ecef2ned) * ned2ros);
	// pose.pose.position.x = ...
        pos_rota[0]=Rotainti[0]*pos_ned[1]+Rotainti[1]*pos_ned[0];
        pos_rota[1]=Rotainti[3]*pos_ned[1]+Rotainti[4]*pos_ned[0];
        pos_rota[2]=-pos_ned[2];

        pose.pose.position.x = pos_rota[0];
        pose.pose.position.y = pos_rota[1];
        pose.pose.position.z = pos_rota[2];
    
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        gps_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
				
}

void callback_hector(const geometry_msgs::PoseStampedConstPtr &hectorpose) {

	geometry_msgs::PoseStamped pose;
	pose.header	= hectorpose->header;
	pose.pose	= hectorpose->pose;

	hector_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
				
}

int main(int argc, char** argv) {

        ros::init(argc, argv, "timestamps");
        ros::NodeHandle node;
        ros::NodeHandle priv_node("~");
        string dir_bag,dir_timestamp;
        char *FileName;

        priv_node.param<std::string>("dir_bag", dir_bag, "/home/du/riegl/figure3/2018-02-12-17-55-58.bag");
        priv_node.param<std::string>("dir_timestamp", dir_timestamp, "/home/du/riegl/figure3/");
	//if(argc < 1)
	//	exit(1);
	int count=0;

	//std::string dir_bag = argv[1];
       // char poseFileName[255];
       // std::string dir_timestamp= argv[2];
        //char FileName[255];
	// read in the data from bagfile

	rosbag::Bag bag(dir_bag);

	rosbag::View view_hector(bag, rosbag::TopicQuery(hector_topic));
	foreach(rosbag::MessageInstance const m, view_hector) {
		if(m.isType<geometry_msgs::PoseStamped>() ) {
			geometry_msgs::PoseStampedConstPtr poseptr = m.instantiate<geometry_msgs::PoseStamped>();
			callback_hector(poseptr);
		}
	}

	rosbag::View view_gps(bag, rosbag::TopicQuery(gps_topic));
	foreach(rosbag::MessageInstance const m,view_gps) {
		if(m.isType<sensor_msgs::NavSatFix>() ) {
			sensor_msgs::NavSatFixConstPtr fixptr = m.instantiate<sensor_msgs::NavSatFix>();
			callback_gps(fixptr);
		}
	}

	// now the timetables are set up and you can look for corresponding stamps.
	// since the gps topic seems to have a lower frequency than the hector topic,
	// look for them in the hector timetable
         // string FileName = poseFileName +'/'+"gps_hector"+".txt";
           const char *posFileName = dir_timestamp.c_str();
          snprintf(FileName,255,"%shectorgps212%.2d.data",posFileName,0);
          ofstream pose_out;
	  pose_out.open(FileName);

          pose_out << "stamp" << " ";
          pose_out << "gps.x" << " ";
          pose_out << "gps.y" << " ";
          pose_out << "gps.z" << " ";
          pose_out << "hector.x" << " ";
          pose_out << "hector.y" << " "; 
          pose_out << "hector.z" << " "; 
          pose_out << endl;


          std::map<double, geometry_msgs::PoseStamped>::iterator gps;
          std::map<double, geometry_msgs::PoseStamped>::iterator gps0;
          
          
          for(gps=gps_timetable.begin();gps!=gps_timetable.end();++gps){
	//for(auto&gps : gps_timetable) {

		double stamp = gps->first;
		geometry_msgs::PoseStamped pose = gps->second;
                gps0=gps_timetable.begin();
                geometry_msgs::PoseStamped pose0 = gps0->second; 
               // pose.pose.position.x=((pose.pose.position.x)-(pose0.pose.position.x));
               // pose.pose.position.y=((pose.pose.position.y)-(pose0.pose.position.y));
               // pose.pose.position.z=((pose.pose.position.z)-(pose0.pose.position.z));
		double dt_min = 1000.0;
		geometry_msgs::PoseStamped pose_min;
                
		// for each gps pose look for the closest hecotr pose wrt time
		//for(auto &hector : hector_timetable) {
                std::map<double, geometry_msgs::PoseStamped>::iterator hector;
                std::map<double, geometry_msgs::PoseStamped>::iterator hector0;
                for(hector=hector_timetable.begin();hector!=hector_timetable.end();++hector){
			double dt = fabs(hector->first - stamp);
			if(dt < dt_min) {
				pose_min = hector->second;
				dt_min = dt;
			}


		}
            
                if(count<2)
              
                  pose_min0=pose_min;
               
                pose_min.pose.position.x=(pose_min.pose.position.x)-(pose_min0.pose.position.x);
                pose_min.pose.position.y=(pose_min.pose.position.y)-(pose_min0.pose.position.y);
                pose_min.pose.position.z=(pose_min.pose.position.z)-(pose_min0.pose.position.z);
                 ".bin" ;
                // now pose and pose_min are a corresponding pair
		// and you can write the poses to a file
		// stamp gps.x gps.y gps.z hector.x ...
               
                cout << "Writing gps and hector to file"<< count++<<endl;
                //cout << stamp <<endl;
                pose_out << pose.header.stamp << " ";
                pose_out << pose.pose.position.x << " ";
                pose_out << pose.pose.position.y << " ";
                pose_out << pose.pose.position.z << " ";

                pose_out << pose_min.pose.position.x << " ";
                pose_out << pose_min.pose.position.y << " ";
                pose_out << pose_min.pose.position.z << " ";
                pose_out << endl; 

                
                
		
	}
                cout << " done." << endl;

           	pose_out.close();
           	pose_out.clear();

		
	// maybe write all gps poses and all hector poses to additional files, for later data processing

	   	bag.close();
               ros::spin();
	   	return 0;

}
