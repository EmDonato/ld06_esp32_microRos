# LD06 LiDAR + ESP32 + micro-ROS Humble

This project integrates an LD06 LiDAR sensor with an ESP32 microcontroller and micro-ROS Humble to publish laser scan data over ROS 2.

## Hardware Components

* **ESP32** microcontroller
* **LD06 LiDAR** sensor (UART interface)
* **micro-ROS Agent** running on host PC or Raspberry Pi

## Software Components

* **Arduino ESP32 framework**
* **micro-ROS Arduino client** for ROS 2 Humble
* **LD06forArduino** library by Inoue Minoru (henjin0): [https://github.com/henjin0/Lidar\_LD06\_for\_Arduino](https://github.com/henjin0/Lidar_LD06_for_Arduino)

## Sketch Overview

```cpp
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "LD06forArduino.h"

#define RESOLUTION 360
#define RX_PIN 16
#define TX_PIN 17
#define BAUDRATE 230400

LD06forArduino lidar;
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;

// Buffers
float ranges[RESOLUTION];
float intensities[RESOLUTION];
bool angle_collected[RESOLUTION];

void resetScanBuffer();
bool isScanComplete();
builtin_interfaces__msg__Time get_now();

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports(wifiNetName, wifiPassword, personalIP, 8888);
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "ld06_node", "", &support);
  rclc_publisher_init_default(
    &scan_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"
  );

  // Configure scan_msg fields
  scan_msg.header.frame_id.data     = (char*)"base_laser";
  scan_msg.header.frame_id.size     = strlen("base_laser");
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
  scan_msg.intensities.data     = intensities;
  scan_msg.intensities.size     = RESOLUTION;
  scan_msg.intensities.capacity = RESOLUTION;

  lidar.Init(RX_PIN);
  resetScanBuffer();
}

void loop() {
  lidar.read_lidar_data();

  if (!lidar.angles.empty()) {
    for (size_t i = 0; i < lidar.angles.size(); ++i) {
      int deg = ((int)round(lidar.angles[i])) % RESOLUTION;
      if (!angle_collected[deg]) {
        float dist_m = lidar.distances[i] / 1000.0f;

        if (dist_m < scan_msg.range_min || dist_m > scan_msg.range_max) {
          ranges[deg] = INFINITY;
        } else {
          ranges[deg] = dist_m;
        }

        intensities[deg]   = lidar.confidences[i];
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
```

## ROS 2 Topics

* **/scan** (`sensor_msgs/LaserScan`): publishes LiDAR scans

## Functions

* **resetScanBuffer()**: clears ranges/intensities arrays and flags
* **isScanComplete()**: checks if full 360° data collected
* **get\_now()**: returns current time as `builtin_interfaces/Time`

## LiDAR Reading Workflow

1. Call `lidar.read_lidar_data()` each loop iteration
2. For each new angle-distance pair:

   * Convert distance to meters
   * Validate against `range_min` and `range_max`
   * Store in `ranges[angle]`, `intensities[angle]`
3. Once buffer full: stamp message, publish, and reset buffer

## Acknowledgements

Special thanks to **Inoue Minoru (henjin0)** for the LD06forArduino library:
[https://github.com/henjin0/Lidar\_LD06\_for\_Arduino](https://github.com/henjin0/Lidar_LD06_for_Arduino)


---

*This sketch enables seamless LiDAR integration for SLAM, obstacle avoidance, or mapping applications under ROS 2.*
