#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

double wrap_angle(double theta) {
  while (theta > 180) theta -= 360;
  while (theta < -180) theta += 360;
  return theta;
}

std::array<float, 4> lemlib::Chassis::calculateOdomOffsets()
{
    std::array<float, 4> ret({-9999, -9999, -9999, -9999});

    // Number of times to test
    int iterations = 10;

    // Our final offsets
    double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

    // Reset all trackers if they exist
    if (sensors.vertical1 != nullptr) sensors.vertical1->reset();
    if (sensors.vertical2 != nullptr) sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();

    for (int i = 0; i < iterations; i++) {
        lateralPID.reset();
        angularPID.reset();
        setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        setPose(0,0,0);
        double imu_start = getPose().theta;
        double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

        // Turn to target at half power
        turnToHeading(target, 2000, {.maxSpeed = 63}, false);
        pros::delay(250);

        // Calculate delta in angle
        double t_delta = lemlib::degToRad(fabs(wrap_angle(getPose().theta - imu_start)));

        // Calculate delta in sensor values that exist
        double l_delta = sensors.vertical1 != nullptr ? sensors.vertical1->getDistanceTraveled() : 0.0;
        double r_delta = sensors.vertical2 != nullptr ? sensors.vertical2->getDistanceTraveled() : 0.0;
        double b_delta = sensors.horizontal1 != nullptr ? sensors.horizontal1->getDistanceTraveled() : 0.0;
        double f_delta = sensors.horizontal2 != nullptr ? sensors.horizontal2->getDistanceTraveled() : 0.0;

        // Calculate the radius that the robot traveled
        l_offset += l_delta / t_delta;
        r_offset += r_delta / t_delta;
        b_offset += b_delta / t_delta;
        f_offset += f_delta / t_delta;
    }

    // Average all offsets
    l_offset /= iterations;
    r_offset /= iterations;
    b_offset /= iterations;
    f_offset /= iterations;

    // Set new offsets to trackers that exist
    if (sensors.vertical1 != nullptr) ret[0] = l_offset;
    if (sensors.vertical2 != nullptr) ret[1] = r_offset;
    if (sensors.horizontal1 != nullptr) ret[2] = b_offset;
    if (sensors.horizontal2 != nullptr) ret[3] = f_offset;

    return ret;
}