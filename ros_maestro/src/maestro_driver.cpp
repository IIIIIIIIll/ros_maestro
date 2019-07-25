#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

int fd; 
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
 
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
 
  return response[0] + 256*response[1];
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, static_cast<unsigned char>(target & 0x7F), static_cast<unsigned char>(target >> 7 & 0x7F)};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
 
void controlCallback(const std_msgs::String::ConstPtr& msg)
{
  //First two chars represent a single number, this is the channel
  //Anythong comes after is the target position  
  std::string temp = msg->data;
  std::string channel = temp.substr(0,2);
  std::string target = temp.substr(2); 

  int ch = std::stoi(channel);
  int tgt = std::stoi(target);
  
  if (ch>5 || ch <0)
  {
    ROS_ERROR("Channel number overbound %d",ch);  //our board has 6 channels
  }
  else  if (tgt>1900 || tgt <1100)
  {
    ROS_ERROR("Target overbound(1100~1900) %d",tgt); //motor and servo's limits NOTE this is manually set to fit specific motor/servo 
  }
  else
  {
    int position = maestroGetPosition(fd, 5);
    ROS_INFO("Current position is %d.", position); //motor/servo position (actual)
    ROS_INFO("Setting target to %d .", tgt);    //input range 
    maestroSetTarget(fd, ch, tgt*4);    //maestro takes input *4 to be the actual position
  } 
}

int main(int argc, char** argv)
{
  // Open the Maestro's virtual COM port.
  //const char * device = "\\\\.\\USBSER000";  // Windows, "\\\\.\\COM6" also works
  const char * device = "/dev/ttyACM0";  // Linux
  //const char * device = "/dev/cu.usbmodem00034567";  // Mac OS X
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
  }
 
#ifdef _WIN32
  _setmode(fd, _O_BINARY);
#else
  struct termios options;
  tcgetattr(fd, &options);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(fd, TCSANOW, &options);
#endif
 

  ros::init(argc, argv, "maestro_driver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("maestro_command",100,controlCallback);
  ros::spin();
    
  close(fd);
}
