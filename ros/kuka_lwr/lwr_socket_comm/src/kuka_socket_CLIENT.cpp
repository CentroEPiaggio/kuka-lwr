#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/message_operations.h>
#include <visualization_msgs/Marker.h>

#include "socket_comm/SocketConnect.h"

// CLIENT SOCKET related headers 
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


// just to remember the order in which the elements should be sent

// cylinder values useful for grasping
// center of reference frame at [x y z]
// rotation matrix  [ ux vx wx ]
//                  [ uy vy wy ]
//                  [ wx wy wz ]
// radius r
// height h

// in rotation matrix form
// float x, y, z, ux, uy, uz, vx, vy, vz, wx, wy, wz, r, h;

// in quartenion orientation form
//float x, y, z, ux, uy, uz, w, r, h;

namespace socket_comm {

class KukaSocketConnector
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    //! Service server for object detection
    ros::ServiceServer kuka_socket_srv_;

    //------------------ Callbacks -------------------

    //! Callback for service calls
    bool serviceCallback(SocketConnect::Request &request, SocketConnect::Response &response);

  public:
    //! Subscribes to and advertises topics; initializes fitter and marker publication flags
    /*! Also attempts to connect to database */
    KukaSocketConnector(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      kuka_socket_srv_ = nh_.advertiseService(nh_.resolveName("kuka_socket_srv"), &KukaSocketConnector::serviceCallback, this);
    }

    //! Empty stub
    ~KukaSocketConnector() {}
};

bool KukaSocketConnector::serviceCallback(SocketConnect::Request &request, SocketConnect::Response &response)
{ 
  ros::Time start_time = ros::Time::now();

  ROS_INFO("A call to the kuka_socket_srv has been requested, configuring the socket...");
  int sock, n;
  unsigned int length;
  struct sockaddr_in server, from;
  struct hostent *hp;
  char buffer[256];
  
  // false unless going through all the code
  response.comm_result.data = false;

  sock= socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) 
  {
    ROS_ASSERT("Error opening socket");
    //return false;
  }

  length = sizeof(server);
  bzero(&server,length);
  struct hostent *hPtr;
  char *remoteHost="192.168.0.101";
  if((hPtr = gethostbyname(remoteHost)) == NULL)
  {
    printf ( "System DNS name resolution not configured properly.\n");
    printf ( "Error number: \n");
    //return false;
  }

  // TODO parametrize the addr and perhaps the port as well to change it at the launch command
  // IF WORKING IN LOCAL MODE UNCOMMENT THIS
  // server.sin_addr.s_addr=INADDR_ANY;
  // IF WORKING IN REMOTE MODE UNCOMMENT THIS
  memcpy((char *)&server.sin_addr, hPtr->h_addr, hPtr->h_length); 
  server.sin_family=AF_INET;
  server.sin_port = htons(2000);

  
  bzero(buffer,256);
  //buffer = request.message_sent.data;
  snprintf(buffer,sizeof buffer, request.message_sent.data.c_str() );
  //float x = 32.6;
  //snprintf(buffer,sizeof buffer,"%fb\n", x);
  ROS_INFO("Sending the message:");
  write(1,buffer,strlen(buffer));
  n=sendto(sock,buffer,strlen(buffer),0,(const struct sockaddr *)&server,length);
  if (n < 0)
  {
    ROS_ASSERT("Error in sendto");
    //return false;
  }
  
  ROS_INFO("Waiting for the receipt acknowledgement...");
  // First received message, so far it is not useful for us
  n = recvfrom(sock,buffer,256,0,(struct sockaddr *)&from, &length);
  if (n < 0)
  {
    ROS_ASSERT("Error in recvfrom");
    //return false;
  }
  write(1,"Server says: ",13);
  write(1,buffer,n);
  write(1,"\n",1);
  
  //ROS_INFO("Ok, then the assumption is that the robot is programmed and is able to perform the movement in 2 seconds");

  //ROS_INFO("strcmp result is %d", strcmp(buffer,"Got your message, the robot is moving...\n"));
  
  // ROS_INFO("Waiting for the result...");
  // // Second received messaga, this is useful for us, write it to the response if required
  // n = recvfrom(sock,buffer,256,0,(struct sockaddr *)&from, &length);
  // if (n < 0)
  // {
  //   ROS_ASSERT("Error in recvfrom");
  //   //return false;
  // }
  // write(1,"Server says: ",13);
  // write(1,buffer,n);
  // write(1,"\n",1);
  
  ROS_INFO("Closing the socket...");

  // close the socket, no need to let it live
  close(sock);
  
  ROS_INFO("Socket closed");

  response.message_recv.data = buffer;
  response.comm_result.data = true;

  return true;
}

} // namespace ros_socket_comm

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "kuka_socket_connector_node");
  ros::NodeHandle nh;

  socket_comm::KukaSocketConnector node(nh);

  ros::spin();
  return 0;
}