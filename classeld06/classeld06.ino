#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <builtin_interfaces/msg/time.h>
#include "LD06forArduino.h"

#define RESOLUTION 360
#define RX_PIN 16 // Pin su cui ricevi dal LD06
#define TX_PIN 17 // Può anche non servire
#define BAUDRATE 230400

LD06forArduino lidar;
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;

// --- Wifi info ---

char wifiNetName[]      = "FASTWEB-USA6DG";//"AutomazioneTesisti";
char wifiPassword[]     = "26KCXAYRSU";//"nicosia456";
char personalIP[]       = "192.168.1.56";//"192.168.0.120";


float ranges[RESOLUTION];
float intensities[RESOLUTION];
bool angle_collected[RESOLUTION];
int count_received = 0;

void resetScanBuffer() {
  for (int i = 0; i < RESOLUTION; ++i) {
    ranges[i] = NAN;
    intensities[i] = 0.0f;
    angle_collected[i] = false;
  }
  count_received = 0;
}

bool isScanComplete() {
  return count_received >= RESOLUTION;
}

builtin_interfaces__msg__Time get_now() {
  builtin_interfaces__msg__Time t;
  int64_t ms = rmw_uros_epoch_millis();
  t.sec = ms / 1000;
  t.nanosec = (ms % 1000) * 1000000;
  return t;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("LD06 + micro-ROS usando libreria LD06forArduino");

  // Init micro-ROS
    set_microros_wifi_transports(wifiNetName, wifiPassword, personalIP, 8888);
  //set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "ld06_node", "", &support);
  rclc_publisher_init_default(
    &scan_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"
  );

  scan_msg.header.frame_id.data = (char*)"base_laser";
  scan_msg.header.frame_id.size = strlen("base_laser");
  scan_msg.header.frame_id.capacity = strlen("base_laser") + 1;
  scan_msg.angle_min       = 0.0f;
  scan_msg.angle_max       = 2.0f * M_PI;
  scan_msg.angle_increment = (2.0f * M_PI) / RESOLUTION;
  scan_msg.scan_time       = 1.0f / 10.0f;
  scan_msg.time_increment  = scan_msg.scan_time / RESOLUTION;
  scan_msg.range_min       = 0.02f;
  scan_msg.range_max       = 12.0f;
  scan_msg.ranges.data     = ranges;
  scan_msg.ranges.size     = RESOLUTION;
  scan_msg.ranges.capacity = RESOLUTION;
  scan_msg.intensities.data = intensities;
  scan_msg.intensities.size = RESOLUTION;
  scan_msg.intensities.capacity = RESOLUTION;

  lidar.Init(RX_PIN);
  resetScanBuffer();
}

void loop() {
  lidar.read_lidar_data();

  if (!lidar.angles.empty()) {
    for (size_t i = 0; i < lidar.angles.size(); ++i) {
    int deg = ((int)round(lidar.angles[i])) % 360;
      if (!angle_collected[deg]) {
        float dist_m = lidar.distances[i] / 1000.0f;

        if (dist_m < scan_msg.range_min || dist_m > scan_msg.range_max)
          ranges[deg] = std::numeric_limits<float>::infinity();
        else
          ranges[deg] = dist_m;

        intensities[deg] = lidar.confidences[i];
        angle_collected[deg] = true;
        count_received++;
      }
    }

    if (isScanComplete()) {
      scan_msg.header.stamp = get_now();
      rcl_publish(&scan_pub, &scan_msg, NULL);
      resetScanBuffer();
    }
  }
}
