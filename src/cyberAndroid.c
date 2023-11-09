/*
 * cyberAndroid.c:
 *	Very simple program to run a robot on the serial port. 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <serial_megapi/serial_megapi.h>

int flag_turning = 0;
float start_turning_angle = 0.0;
int gyro_received_once = 0;

void drive_forward(const int fd, const int motor_left, const int motor_right, const int wheel_speed){
  set_speed(fd, motor_left, -wheel_speed);
  set_speed(fd, motor_right, wheel_speed);
}

void turn_left(const int fd, const int motor_left, const int motor_right, const int wheel_speed){
  set_speed(fd, motor_left, wheel_speed);
  set_speed(fd, motor_right, wheel_speed);
}

void turn_right(const int fd, const int motor_left, const int motor_right, const int wheel_speed){
  set_speed(fd, motor_left, -wheel_speed);
  set_speed(fd, motor_right, -wheel_speed);
}

void stop_motors(const int fd){
  set_speed(fd, 1, 0);
  set_speed(fd, 2, 0);
  set_speed(fd, 3, 0);
  set_speed(fd, 4, 0);
}

void move_my_robot(const int fd, float distance, float yaw, const int motor_left, const int motor_right, const int wheel_speed){
  if(distance > 20){
    drive_forward(fd, motor_left, motor_right, wheel_speed);
  } else {
    stop_motors(fd);
  }
}

void turn_my_robot(const int fd, float yaw, const int motor_left, const int motor_right, const int wheel_speed, const float angle_degree){
  if(!flag_turning && gyro_received_once){
    start_turning_angle = yaw;
    flag_turning = 1;
  }

  double epsilon = abs(yaw - start_turning_angle);

  printf (" Epsilon : %f / aim_angle : %f", epsilon, angle_degree) ;

  if(epsilon < abs(angle_degree)){
    // then we want to turn
    if(angle_degree > 0){
      turn_left(fd, motor_left, motor_right, wheel_speed);
    } else {
      turn_right(fd, motor_left, motor_right, wheel_speed);
    }
  } else {
    // we want to stop
    stop_motors(fd);
    // flag_turning = 0;
  }
  
  
}

int main ()
{
  int fd;
  int count=0;
  unsigned int nextTime ;

  if ((fd = init_serial("/dev/ttyAMA0", 115200)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  const int uss_port = 0x7;

  const int motor_left_port = 3;
  const int motor_right_port = 2;
  const int max_speed = 255;
  const int min_speed = 15;
  const int set_speed = 30;

  const float desire_turn_angle = -90.0;

  float front_ditance = 0.0;
  float gyro_yaw = 0.0;

  nextTime = millis () + 300 ;

  for (;;)
  {
    if (millis () > nextTime)
    {
      printf ("\nOut: %3d / %d / ", count, fd) ;
      fflush (stdout) ;

      if(0 == count % 2){
        request_gyro(fd, GYRO_ALL_AXES);
      } else {
        request_uss(fd, uss_port);
      }
      
      // move_my_robot(fd, front_ditance, gyro_yaw, motor_left_port, motor_right_port, set_speed);
      turn_my_robot(fd, gyro_yaw, motor_left_port, motor_right_port, set_speed, desire_turn_angle);
      
      nextTime += 100 ;
      ++count ;
    }

    if(is_ultrasonic_new_data(uss_port)){
      front_ditance = get_uss_cm(uss_port);
      printf(" / uss_cm : %f \n", front_ditance);
    }
      
    if(is_gyro_new_data()){
      gyro_received_once = 1;
      gyro_yaw = get_gyro_yaw();
      printf(" / Gyro : %f \n", gyro_yaw);
    }

  }

  printf ("\n") ;
  return 0 ;
}
