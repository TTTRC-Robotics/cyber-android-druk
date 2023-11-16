/*
 * cornerChallenge.c:
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

int fd_ = -1;

const int motor_left_ = 3;
const int motor_right_ = 2;

const int uss_port_ = 0x7;

const int max_speed_ = 255;
const int min_speed_ = 15;
const int set_speed_ = 30;

float front_distance_ = 0.0;
float yaw_ = 0.0;
float last_yaw_ = 0.0;


int turn_count = 0;
int flag_driving = 0;
int flag_turning = 0;
float start_turning_angle = 0.0;
int gyro_received_once = 0;
int uss_received_once = 0;

int movement_phase = 0;

#define BUFFER_SIZE 5
float circularYaw[BUFFER_SIZE] = {0.0}; // Initialize the buffer with zeros
int currentIndex = 0;

void updateBuffer(float buffer[], float newValue, int *index) {
    buffer[*index] = newValue;
    *index = (*index + 1) % BUFFER_SIZE;
}

int compare(const void *a, const void *b) {
    return (*(float*)a > *(float*)b) - (*(float*)a < *(float*)b);
}

float calculateMedian(float buffer[]) {
  float sortedBuffer[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    sortedBuffer[i] = buffer[i];
  }

  // Using the qsort function to sort the array
  qsort(sortedBuffer, BUFFER_SIZE, sizeof(float), compare);

  // Calculating the median
  return sortedBuffer[BUFFER_SIZE / 2];
}

void drive_forward(const int wheel_speed){
  set_speed(fd_, motor_left_, -wheel_speed);
  set_speed(fd_, motor_right_, wheel_speed);
}

void drive_backward(const int wheel_speed){
  set_speed(fd_, motor_left_, wheel_speed);
  set_speed(fd_, motor_right_, -wheel_speed);
}

void turn_left(const int wheel_speed){
  set_speed(fd_, motor_left_, wheel_speed);
  set_speed(fd_, motor_right_, wheel_speed);
}

void turn_right(const int wheel_speed){
  set_speed(fd_, motor_left_, -wheel_speed);
  set_speed(fd_, motor_right_, -wheel_speed);
}

void stop_motors(){
  set_speed(fd_, 1, 0);
  set_speed(fd_, 2, 0);
  set_speed(fd_, 3, 0);
  set_speed(fd_, 4, 0);
}

void move_backward_until(const int wheel_speed, const float distance_threshold){
  if(!flag_driving){
    flag_driving = 1;
    printf("Start Driving Backward %f ", front_distance_);
  }
  if(front_distance_ < distance_threshold){
    drive_backward(wheel_speed);
  } else {
    stop_motors();
    movement_phase += 1;
    flag_driving = 0;
    printf("Stop Driving Backward %f\n", front_distance_);
  }
}

void move_forward_until(const int wheel_speed, const float distance_threshold){
  if(!flag_driving){
    flag_driving = 1;
    printf("Start Driving Forward %f ", front_distance_);
  }
  if(front_distance_ > distance_threshold){
    drive_forward(wheel_speed);
  } else {
    stop_motors();
    movement_phase += 1;
    flag_driving = 0;
    printf("Stop Driving Forward %f\n", front_distance_);
  }
}

void turn_my_robot(const int wheel_speed, const float angle_degree){
  if(abs(yaw_ - last_yaw_) > 350){
    if( yaw_ - last_yaw_ > 0){
      turn_count -= 1; 
    } else {
      turn_count += 1;
    }
  }

  updateBuffer(circularYaw, yaw_, &currentIndex);
  

  float real_yaw = yaw_ + turn_count * 360;
  
  if(!flag_turning){
    start_turning_angle = calculateMedian(circularYaw) + turn_count * 360 ;
    flag_turning = 1;

    printf("Start Turning %f - Yaw : %f", angle_degree, start_turning_angle);
  }

  double delta_angle = real_yaw - start_turning_angle;
  double epsilon = delta_angle - angle_degree;
  const double tolerance_degree = 2.5;

  // printf (" Delta_angle : %f / aim_angle : %f / Epsilon : %f / real_yaw : %f ", delta_angle, angle_degree, epsilon, real_yaw) ;

  if(abs(epsilon) > tolerance_degree ){
    // then we want to turn
    if(epsilon < 0){
      turn_left(wheel_speed);
    } else {
      turn_right(wheel_speed);
    }
  } else {
    // we want to stop
    stop_motors();
    flag_turning = 0;
    movement_phase += 1;
    printf("Stop Turning %f - Yaw : %f \n", angle_degree, real_yaw);
  }
  
  last_yaw_ = yaw_;
}

void move_my_robot(){
  switch(movement_phase) {
    case 0:
      // we want to drive forward until the wall
      move_forward_until(set_speed_, 20.0);
      break;
    case 1:
      // We want to turn 90 degrees
      turn_my_robot(set_speed_, 90.0);
      break;
    case 2:
      // we want to drive forward until the wall
      move_forward_until(set_speed_, 20.0);
      break;
    case 3:
      // we want to drive backward until the wall
      move_backward_until(set_speed_, 60.0);
      break;
    case 4:
      // We want to turn -90 degrees
      turn_my_robot(set_speed_, -90.0);
      break;
    case 5:
      // we want to drive backward until the wall
      move_backward_until(set_speed_, 145.0);
      break;
    case 6:
      // We want to turn 90 degrees
      turn_my_robot(set_speed_, 90.0);
      break;
    case 7:
      // we want to drive forward until the wall
      move_forward_until(set_speed_, 20.0);
      break;
    case 8:
      // we want to drive backward to the middle
      move_backward_until(set_speed_, 40.0);
      break;
    case 9:
      // We want to turn -90 degrees
      turn_my_robot(set_speed_, -90.0);
      break;
    case 10:
      // we want to drive forward until the middle
      move_forward_until(set_speed_, 90.0);
      break;
    case 11:
      printf("Done\n");
      movement_phase += 1;
      break;
    default:
      break;
    }
}

int main ()
{
  int count=0;
  unsigned int nextTime ;

  if ((fd_ = init_serial("/dev/ttyAMA0", 115200)) < 0){
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  nextTime = millis () + 300 ;

  for (;;)
  {
    if (millis () > nextTime)
    {
      // printf ("\nOut: %3d / ", count) ;
      fflush (stdout) ;

      if(0 == count % 2){
        request_gyro(fd_, GYRO_ALL_AXES);
      } else {
        request_uss(fd_, uss_port_);
      }
      
      if(uss_received_once && gyro_received_once){
        move_my_robot();
      }
      
      nextTime += 100 ;
      ++count ;
    }

    if(is_ultrasonic_new_data(uss_port_)){
      uss_received_once = 1;
      front_distance_ = get_uss_cm(uss_port_);
      // printf(" / uss_cm : %f \n", front_distance_);
    }
      
    if(is_gyro_new_data()){
      gyro_received_once = 1;
      yaw_ = get_gyro_yaw();
      // printf(" / Gyro : %f \n", yaw_);
    }

  }

  printf ("\n") ;
  return 0 ;
}
