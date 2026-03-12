#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::driftToPose(float x, float y, float theta, int timeout, DriftToPoseParams params, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;

    if (async) {
        pros::Task task([&]() { driftToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    // 1. Calculate the physics-based speed cap
    // V = sqrt(2 * a * d). This is the max speed the robot can have 
    // to stop within driftDistance given the friction deceleration.
    float max_drift_velocity = sqrt(2 * params.friction * params.driftDistance);

    lateralPID.reset();
    angularPID.reset();
    Timer timer(timeout);
    Pose target(x, y);
    bool isDrifting = false;
    float prevLateralOut = 0;

    while (!timer.isDone() && this->motionRunning) {
        Pose pose = getPose(true, true);
        float distTarget = pose.distance(target);
        
        // --- PHASE 1: LINEAR APPROACH ---
        if (!isDrifting && distTarget > params.driftDistance) {
            float lateralError = distTarget * cos(angleError(pose.theta, pose.angle(target)));
            float angularError = angleError(params.forwards ? pose.theta : pose.theta + M_PI, pose.angle(target));

            float lateralOut = lateralPID.update(lateralError);
            
            // 2. Apply the physics cap
            // We ensure lateralOut never exceeds the velocity friction can handle
            if (fabs(lateralOut) > max_drift_velocity) {
                lateralOut = sgn(lateralOut) * max_drift_velocity;
            }

            float angularOut = angularPID.update(radToDeg(angularError));
            lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);
            
            drivetrain.leftMotors->move(lateralOut + angularOut);
            drivetrain.rightMotors->move(lateralOut - angularOut);
            prevLateralOut = lateralOut;
        } 
        // --- PHASE 2: THE DRIFT/SWING ---
        else {
            isDrifting = true;
            float deltaTheta = angleError(theta, pose.theta, false);
            
            // Exit if heading is reached
            angularLargeExit.update(deltaTheta);
            angularSmallExit.update(deltaTheta);
            if (angularLargeExit.getExit() || angularSmallExit.getExit()) break;

            float motorPower = angularPID.update(deltaTheta);

            // Determine swing side based on error direction
            if (deltaTheta > 0) { // Turning Left
                drivetrain.rightMotors->move(motorPower);
                drivetrain.leftMotors->brake(); 
            } else { // Turning Right
                drivetrain.leftMotors->move(-motorPower);
                drivetrain.rightMotors->brake();
            }
        }
        pros::delay(10);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    this->endMotion();
}