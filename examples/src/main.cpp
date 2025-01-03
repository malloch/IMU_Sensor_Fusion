#include <cmath>
#include <Arduino.h>
#include <iostream>
#include <unistd.h>

int i;
#define USEC_TO_MSEC 0.001
#include "imu_orientation.h"

// put function declarations here:
void benchmarkSensorFusion(int num_loops);

void benchmarkSensorFusion(int num_loops) {
  IMU_Orientation sensor_fusion;
  int count = num_loops;
  long start, end, elapsed_time;
  float time_per_loop;
  start = micros();
  while (count--) {
      sensor_fusion.setAccelerometerValues(0, 1, 0);
      sensor_fusion.setGyroscopeRadianValues(0, 0, 0, 0.001);
      sensor_fusion.setMagnetometerValues(0, 1, 2);
      sensor_fusion.update();
  }
  end = micros();
  elapsed_time = end-start;
  time_per_loop = float(elapsed_time) / float(num_loops);
  elapsed_time = elapsed_time * USEC_TO_MSEC;


  // Print time
  std::cout << "Iterations: " << num_loops << " total_time: " << elapsed_time << "ms"<< " unit_time: " << time_per_loop << "us"<< std::endl;
}

void setup() {
  // Benchmark the code
  std::cout << "Benchmark ESP32S3 Sensor Fusion Performance " << std::endl;
  delay(500);
}

void loop() {
  std::cout << "Start Test " << i << std::endl;
  i++;
  // put your main code here, to run repeatedly:
  benchmarkSensorFusion(100000);
  delay(5000);
}
