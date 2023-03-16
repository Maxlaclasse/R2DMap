#include <iostream>
#include <ros/ros.h>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "lipkg.h"


using namespace std;

const int UDP_PORT = 8085;
const int BUFFER_SIZE = 47;
static int nbr_pub = 0;
int main(int argc, char **argv) {

    ros::init(argc, argv, "ldldiar_publisher");
    ros::NodeHandle nh;  // create a ROS Node
    ros::NodeHandle n("~");
    std::string product_name = "LDLiDAR_LD06";
    std::string topic_name = "/scan";
    std::string frame_id = "laser_frame";
    bool laser_scan_dir = true;
    bool enable_angle_crop_func = false;
    double angle_crop_min = 135.0;
    double angle_crop_max = 135.0;

    n.getParam("product_name", product_name);
    n.getParam("topic_name", topic_name);
    n.getParam("frame_id", frame_id);
    n.getParam("laser_scan_dir", laser_scan_dir);
    n.getParam("enable_angle_crop_func", enable_angle_crop_func);
    n.getParam("angle_crop_min", angle_crop_min);
    n.getParam("angle_crop_max", angle_crop_max);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        cerr << "Failed to create socket" << endl;
        return -1;
    }
    LiPkg *lidar = new LiPkg(frame_id, laser_scan_dir, enable_angle_crop_func, angle_crop_min, angle_crop_max);

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        cerr << "Failed to bind socket to port " << UDP_PORT << endl;
        close(sock);
        return -1;
    }

    cout << "Listening for UDP packets on port " << UDP_PORT << endl;

    Points2D laser_scan;

    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic
  
    ros::Rate r(10); //10hz

    while (true) {

        while (ros::ok()) {
            char buffer[BUFFER_SIZE];
            struct sockaddr_in client_addr;
            socklen_t client_addr_len = sizeof(client_addr);
            int num_bytes = recvfrom(sock, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&client_addr, &client_addr_len);
            if (num_bytes < 0) {
                cerr << "Failed to receive data" << endl;
            } else {
                if (lidar->Parse((uint8_t*)buffer, sizeof(buffer))) {

                    if(lidar->AssemblePacket()) {

                        //laser_scan = lidar->GetData(); 
            
                        lidar_pub.publish(lidar->GetLaserScan());  
                        cout << "lidar published" << endl;   
                        nbr_pub++;
                        r.sleep();              

                    }

                
                }
            }
            }
        }




    close(sock);
    return 0;
}

