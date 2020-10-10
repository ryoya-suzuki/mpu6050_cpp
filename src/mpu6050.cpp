#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cmath>

using namespace std;

#define ACCEL_X_OUT 0x3b
#define ACCEL_Y_OUT 0x3d
#define ACCEL_Z_OUT 0x3f
#define TEMP_OUT 0x41
#define GYRO_X_OUT 0x43 
#define GYRO_Y_OUT 0x45
#define GYRO_Z_OUT 0x47
#define LPF_ADDR 0x1a

#define PWR_MGMT_1 0x6B  //PWR_MGMT_1
#define PWR_MGMT_2 0x6c  //PWR_MGMT_2
#define DEV_ADDR 0x68    // I2C

#define RAD_TO_DEG 57.324

float read_word_2c(int fd, int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

void calcEuler(float x, float y, float z, float *theta, float *psi, float *phi){
  *theta = atan(x / sqrt(y * y + z * z));
  *psi = atan(y / sqrt(x*x + z*z));
  *phi = atan(sqrt(x * x + y * y) / z);
}

int main(int argc, char **argv) {

  // Connect to device.
  int fd = wiringPiI2CSetup(DEV_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);
  // Set LRF
  wiringPiI2CWriteReg16(fd, LPF_ADDR, 0x03);

  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu", 1);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(1);

  float ax,ay,az,gx,gy,gz,roll,pitch,yaw;
  tf::Quaternion q;

  // Publish in loop.
  while(ros::ok()) {
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "mpu6050";  // no frame

    // Read gyroscope values.
    // At default sensitivity of 250deg/s we need to scale by 131.
    gx = read_word_2c(fd, GYRO_X_OUT) / 131.0;
    gy = read_word_2c(fd, GYRO_Y_OUT) / 131.0;
    gz = read_word_2c(fd, GYRO_Z_OUT) / 131.0;
    

    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
    //const float la_rescale = 16384.0 / 9.807;
    ax = read_word_2c(fd, ACCEL_X_OUT) / 16384.0;
    ay = read_word_2c(fd, ACCEL_Y_OUT) / 16384.0;
    az = read_word_2c(fd, ACCEL_Z_OUT) / 16384.0;

    calcEuler(ax, ay, az, &roll, &pitch, &yaw);
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    imu.orientation.x = q[0];
    imu.orientation.y = q[1];
    imu.orientation.z = q[2];
    imu.orientation.w = q[3];

    imu.angular_velocity.x = gx;
    imu.angular_velocity.y = gy;
    imu.angular_velocity.z = gz;
    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;

    // Pub & Broadcast, sleep.
    pub.publish(imu);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mpu6050", "map"));
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// submodule test
