#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <stdio.h>
#include <string.h>
#include <cstdint>
#include <iostream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>




class trinamic_servo : public hardware_interface::RobotHW
{
public:
  
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  uint8_t write_buf[4];
  uint8_t read_buf[4];
  struct termios tty;
  int serial_port;
  uint32_t target;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double loop_hz_;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;


  trinamic_servo(ros::NodeHandle& nh) : nh_(nh)
  { 

   controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
   //nh_.param("/trinamic/loop_hz", loop_hz_, 0.1);     
   loop_hz_ = 1000.0;
   ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
   non_realtime_loop_ = nh_.createTimer(update_freq, &trinamic_servo::update, this);
   init_serial();
   init_hw();
  }


  ~trinamic_servo(){

  }

  void init_hw(){
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_servo1("servo1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_servo1);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_servo1(jnt_state_interface.getHandle("servo1"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_servo1);

   registerInterface(&jnt_pos_interface);

  }

  void init_serial(){ 

   //connect to serial port

    serial_port = open("/dev/ttyACM0", O_RDWR);
   

  // Read in existing settings, and handle any error
	  if(tcgetattr(serial_port, &tty) != 0) {
	      std::cout<< "Error " <<  errno << " from tcgetattr: " << strerror(errno) << std::endl;
	      return ;
	  }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);
	  
    // Save tty settings, also checking for error
	  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	      std::cout<< "Error " <<  errno << " from tcgetattr: " << strerror(errno) << std::endl;
	      return ;
	  }
    std::cout<< "Serial initialized!"<< std::endl;

  }

  void update(const ros::TimerEvent& e) 
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    trinamic_servo::read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    trinamic_servo::write();
  }

  void write(){

   	target = int32_t(cmd[0]/0.00002396844);
    
    //std::cout << pos[0] << std::endl;

	  write_buf[0] = (target & 0xFF000000) >> 24;
  	write_buf[1] = (target & 0x00FF0000) >> 16;
  	write_buf[2] = (target & 0x0000FF00) >> 8;
  	write_buf[3] = (target & 0x000000FF);
  	::write(serial_port, write_buf, 4);
   
  }

  void read(){
 	
   	memset(&read_buf, '\0', sizeof(read_buf));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
  	int num_bytes = ::read(serial_port, &read_buf, sizeof(read_buf));

  	pos[0] = (read_buf[0] << 24 | read_buf[1] << 16 | read_buf[2] << 8 | read_buf[3]) * 0.00002396844;
    //std::cout << pos[0] << std::endl;
  }

};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "trinamic_interface");
  ros::NodeHandle nh;
  trinamic_servo servo(nh);
 
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
 
  return 0;
}
