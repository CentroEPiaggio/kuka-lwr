#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <sstream>

// SERVER SOCKET related headers 
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("robot_moving", 1000);
  // The robot moving can be true or false

  // Set the freq of the main loop
  ros::Rate loop_rate(1);

  // Initialize the standard socket server that will receive the info from external programs and publish it to the topic stated above
  int sock, length, n;
  socklen_t fromlen;
  struct sockaddr_in server;
  struct sockaddr_in from;
  char buf[1024];
  
  sock=socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) 
  {
    ROS_ASSERT("Error opening socket");
  }
  
  length = sizeof(server);
  bzero(&server,length);
  server.sin_family=AF_INET;
  server.sin_addr.s_addr=INADDR_ANY;
  server.sin_port=htons(2000);
  
  if (bind(sock,(struct sockaddr *)&server,length)<0)
  {
    ROS_ASSERT("Error binding the socket");
  }

  fromlen = sizeof(struct sockaddr_in);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

    n = recvfrom(sock,buf,1024,0,(struct sockaddr *)&from,&fromlen);
    if (n < 0)
    {
      ROS_ASSERT("Error in recvfrom");
    }
    write(1,"Received a data: ",17);
    write(1,buf,n);


    n = sendto(sock,"Got your message, the robot is moving...\n",41,0,(struct sockaddr *)&from,fromlen);
    if (n  < 0) 
    {
      ROS_ASSERT("Error in sendto");
    }

    // MOVING THE ROBOT
    ROS_INFO("Moving the robot...");
    loop_rate.sleep();
    // // STOPPING THE ROBOT

    // // If moving the robot succeded, send "Done"
    // n = sendto(sock,"Done",4,0,(struct sockaddr *)&from,fromlen);
    // if (n  < 0) 
    // {
    //   ROS_ASSERT("Error in sendto");
    // }
    
    // Final message
    //ROS_INFO("The robot is stopped.");

    // If moving the robot didn't succed, send " Fail"
//    n = sendto(sock,"Fail",4,0,(struct sockaddr *)&from,fromlen);
//    if (n  < 0) 
//    {
//      ROS_ASSERT("Error in sendto");
//    }

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "done" << count;
    //msg.data = ss.str();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ++count;

    ros::spinOnce();
  }


  return 0;
}