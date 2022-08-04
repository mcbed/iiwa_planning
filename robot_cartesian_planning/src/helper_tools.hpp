#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


std::vector<geometry_msgs::msg::Pose> csv2path(std::string path_filename, std::vector<double> shift = {0,0,0}, double scale = 1){
    std::ifstream csvFile;
    std::string line;
    std::string delimiters = ",";
    std::vector<std::string> splitLine;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose waypoint, past_waypoint;

    if(shift.size() != 3){
        std::cerr << "shift vector size size needs to be 3 " << std::endl;
        return waypoints;
    }

    double offsetX = shift[0];
    double offsetY = shift[1];
    double offsetZ = shift[2];

    csvFile.open(path_filename);

    if(csvFile){
        std::getline(csvFile,line);
        while(std::getline(csvFile,line)){
            boost::split(splitLine, line, boost::is_any_of(delimiters));
            waypoint.position.x = (std::stod(splitLine[0]))*scale+offsetX;
            waypoint.position.y = (std::stod(splitLine[1]))*scale+offsetY;
            waypoint.position.z = (std::stod(splitLine[2]))*scale+offsetZ;
            waypoint.orientation.w = std::stod(splitLine[3]);
            waypoint.orientation.x = std::stod(splitLine[4]);
            waypoint.orientation.y = std::stod(splitLine[5]);
            waypoint.orientation.z = std::stod(splitLine[6]);
            if (waypoint != past_waypoint){
                waypoints.push_back(waypoint);
                past_waypoint = waypoint;
            }
        }
    }
  csvFile.close();

  return waypoints;
  
}
// ------------------------------------------------------------------------------------
std::vector<geometry_msgs::msg::Pose> apt2path(std::string path_filename, std::vector<double> shift = {0,0,0}, double scale = 1){
    std::ifstream aptFile;
    std::string line;
    std::string delimiters = ",/$";
    std::vector<std::string> splitLine;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose waypoint, past_waypoint;

    if(shift.size() != 3){
        std::cerr << "shift vector size size needs to be 3 " << std::endl;
        return waypoints;
    }

    double offsetX = shift[0];
    double offsetY = shift[1];
    double offsetZ = shift[2];

    aptFile.open(path_filename);

    if(aptFile){
        while(std::getline(aptFile,line)){
            boost::split(splitLine, line, boost::is_any_of(delimiters));
            if(splitLine[0] == "GOTO "){
                waypoint.position.x = (std::stod(splitLine[1]))*scale+offsetX;
                waypoint.position.y = (std::stod(splitLine[2]))*scale+offsetY;
                waypoint.position.z = (std::stod(splitLine[3]))*scale+offsetZ;
                Eigen::Quaternion<double> qu(0,0,0,1);
                Eigen::Quaternion<double> qv(1,0,0,0);
                std::size_t found = line.find("$");
                if (found!=std::string::npos){
                    std::getline(aptFile,line);

                    boost::split(splitLine, line, boost::is_any_of(delimiters));
                    qv.w() = 0;
                    qv.x() = std::stod(splitLine[0]);
                    qv.y() = std::stod(splitLine[1]);
                    qv.z() = std::stod(splitLine[2]);
                }
                
                Eigen::Quaternion<double> q = qu*qv;
                q.w() = 1-q.w();
                q.normalize();

                    
                Eigen::Quaternion<double> qrot;  
                qrot = Eigen::AngleAxis<double>(M_PI,  Eigen::Vector3d(1,0,0));

                q = q*qrot;
                
                waypoint.orientation.w = q.w();
                waypoint.orientation.x = q.x();
                waypoint.orientation.y = q.y();
                waypoint.orientation.z = q.z();

                if (waypoint != past_waypoint){
                    waypoints.push_back(waypoint);
                    past_waypoint = waypoint;
                }   
            }
        }
    }
  aptFile.close();

  return waypoints;
  
}
