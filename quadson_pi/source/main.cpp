#include "AHRS.hpp"
#include "can_interface.h"
#include "quadson.h"
#include <Common/Util.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <can_protocol.h>

int main() {
  if (check_apm()) { // check Navio sensor is not running
    return 1;
  }

  //--------------------------- Network setup -------------------------------

  Can_interface can_dev;
  AHRS ahrs;
  Quadson quadson(&can_dev);

  quadson.scan();
  // sleep(1);
  //-------------------- Setup gyroscope offset -----------------------------

  // ahrs.setGyroOffset();

  //------------------------ Read Euler angles ------------------------------
  // Orientation data

  // -------- Read raw measurements from the MPU and update AHRS --------------
  // float roll, pitch, yaw;

  // ahrs.getEuler(&roll, &pitch, &yaw);
  // while (1) {
  //   ahrs.update();
  //   quadson.update();
  //   sleep(0.5);
  // }

  // -------------------- Testing section -----------------------------------
  quadson.moving_test();
  
  can_dev.can_close();
  return 0;
}