#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"



#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

// Assuming lemlib::Pose is available in your project environment
// #include "lemlib/api.hpp" 

namespace DubinsMath {

    const double PI = 3.14159265358979323846;

    struct PathPoint
    {
        lemlib::Pose pose;
        float v;
        float t;
    };

    struct SegmentLengths {
        double len[3];
        std::string mode;
        bool valid;
    };

    double mod2pi(double theta) {
        double mod_angle = std::fmod(theta, 2.0 * PI);
        if (mod_angle < 0) {
            mod_angle += 2.0 * PI;
        }
        return mod_angle;
    }

    double angle_mod(double x) {
        double mod_angle = std::fmod(x + PI, 2.0 * PI);
        if (mod_angle < 0) {
            mod_angle += 2.0 * PI;
        }
        return mod_angle - PI;
    }

    SegmentLengths LSL(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double p_squared = 2.0 + d * d - (2.0 * cos_ab) + (2.0 * d * (sin_a - sin_b));
        if (p_squared < 0) return {{0, 0, 0}, "LSL", false};
        
        double tmp = std::atan2((cos_b - cos_a), d + sin_a - sin_b);
        double d1 = mod2pi(-alpha + tmp);
        double d2 = std::sqrt(p_squared);
        double d3 = mod2pi(beta - tmp);
        
        return {{d1, d2, d3}, "LSL", true};
    }

    SegmentLengths RSR(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double p_squared = 2.0 + d * d - (2.0 * cos_ab) + (2.0 * d * (sin_b - sin_a));
        if (p_squared < 0) return {{0, 0, 0}, "RSR", false};
        
        double tmp = std::atan2((cos_a - cos_b), d - sin_a + sin_b);
        double d1 = mod2pi(alpha - tmp);
        double d2 = std::sqrt(p_squared);
        double d3 = mod2pi(-beta + tmp);
        
        return {{d1, d2, d3}, "RSR", true};
    }

    SegmentLengths LSR(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double p_squared = -2.0 + d * d + (2.0 * cos_ab) + (2.0 * d * (sin_a + sin_b));
        if (p_squared < 0) return {{0, 0, 0}, "LSR", false};
        
        double d1 = std::sqrt(p_squared);
        double tmp = std::atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - std::atan2(-2.0, d1);
        double d2 = mod2pi(-alpha + tmp);
        double d3 = mod2pi(-mod2pi(beta) + tmp);
        
        return {{d2, d1, d3}, "LSR", true};
    }

    SegmentLengths RSL(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double p_squared = d * d - 2.0 + (2.0 * cos_ab) - (2.0 * d * (sin_a + sin_b));
        if (p_squared < 0) return {{0, 0, 0}, "RSL", false};
        
        double d1 = std::sqrt(p_squared);
        double tmp = std::atan2((cos_a + cos_b), (d - sin_a - sin_b)) - std::atan2(2.0, d1);
        double d2 = mod2pi(alpha - tmp);
        double d3 = mod2pi(beta - tmp);
        
        return {{d2, d1, d3}, "RSL", true};
    }

    SegmentLengths RLR(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double tmp = (6.0 - d * d + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0;
        if (std::abs(tmp) > 1.0) return {{0, 0, 0}, "RLR", false};
        
        double d2 = mod2pi(2.0 * PI - std::acos(tmp));
        double d1 = mod2pi(alpha - std::atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0);
        double d3 = mod2pi(alpha - beta - d1 + d2);
        
        return {{d1, d2, d3}, "RLR", true};
    }

    SegmentLengths LRL(double alpha, double beta, double d) {
        double sin_a = std::sin(alpha), sin_b = std::sin(beta);
        double cos_a = std::cos(alpha), cos_b = std::cos(beta);
        double cos_ab = std::cos(alpha - beta);
        
        double tmp = (6.0 - d * d + 2.0 * cos_ab + 2.0 * d * (-sin_a + sin_b)) / 8.0;
        if (std::abs(tmp) > 1.0) return {{0, 0, 0}, "LRL", false};
        
        double d2 = mod2pi(2.0 * PI - std::acos(tmp));
        double d1 = mod2pi(-alpha - std::atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0);
        double d3 = mod2pi(mod2pi(beta) - alpha - d1 + mod2pi(d2));
        
        return {{d1, d2, d3}, "LRL", true};
    }

    void interpolate(double length, char mode, double max_curvature, double origin_x, double origin_y, double origin_yaw, 
                     std::vector<double>& path_x, std::vector<double>& path_y, std::vector<double>& path_yaw) {
        if (mode == 'S') {
            path_x.push_back(origin_x + length / max_curvature * std::cos(origin_yaw));
            path_y.push_back(origin_y + length / max_curvature * std::sin(origin_yaw));
            path_yaw.push_back(origin_yaw);
        } else { // curve
            double ldx = std::sin(length) / max_curvature;
            double ldy = 0.0;
            if (mode == 'L') {
                ldy = (1.0 - std::cos(length)) / max_curvature;
            } else if (mode == 'R') {
                ldy = (1.0 - std::cos(length)) / -max_curvature;
            }

            double gdx = std::cos(-origin_yaw) * ldx + std::sin(-origin_yaw) * ldy;
            double gdy = -std::sin(-origin_yaw) * ldx + std::cos(-origin_yaw) * ldy;
            
            path_x.push_back(origin_x + gdx);
            path_y.push_back(origin_y + gdy);

            if (mode == 'L') {
                path_yaw.push_back(origin_yaw + length);
            } else if (mode == 'R') {
                path_yaw.push_back(origin_yaw - length);
            }
        }
    }

} // namespace DubinsMath


/**
 * Generates an optimal Dubins path with embedded velocity and time profiles
 * @param s_x Start X position
 * @param s_y Start Y position
 * @param s_yaw Start Yaw angle (radians)
 * @param g_x Goal X position
 * @param g_y Goal Y position
 * @param g_yaw Goal Yaw angle (radians)
 * @param curvature The curvature limit (1.0 / radius)
 * @param step_size The spacing between generated points on the path
 * @param minSpeed Minimum velocity limit (0-127)
 * @param maxSpeed Maximum velocity limit (0-127)
 * @param horizontalDrift Drift constant for cornering speeds
 * @return A vector of DubinsMath::PathPoint objects
 */
std::vector<DubinsMath::PathPoint> generate_dubins_path(
    double s_x, double s_y, double s_yaw, 
    double g_x, double g_y, double g_yaw, 
    double curvature, double step_size,
    float minSpeed, float maxSpeed, float horizontalDrift, float slowDist) 
{
    using namespace DubinsMath;

    // 1. Calculate local goal (x, y, yaw) relative to start
    double dx = g_x - s_x;
    double dy = g_y - s_y;
    
    double local_goal_x = dx * std::cos(s_yaw) + dy * std::sin(s_yaw);
    double local_goal_y = -dx * std::sin(s_yaw) + dy * std::cos(s_yaw);
    double local_goal_yaw = g_yaw - s_yaw;

    // 2. Setup path planning from origin
    double d = std::hypot(local_goal_x, local_goal_y) * curvature;
    double theta = mod2pi(std::atan2(local_goal_y, local_goal_x));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(local_goal_yaw - theta);

    double best_cost = std::numeric_limits<double>::infinity();
    SegmentLengths best_seg;

    std::vector<SegmentLengths> planners = {
        LSL(alpha, beta, d), RSR(alpha, beta, d), LSR(alpha, beta, d),
        RSL(alpha, beta, d), RLR(alpha, beta, d), LRL(alpha, beta, d)
    };

    for (const auto& seg : planners) {
        if (!seg.valid) continue;
        double cost = std::abs(seg.len[0]) + std::abs(seg.len[1]) + std::abs(seg.len[2]);
        if (cost < best_cost) {
            best_cost = cost;
            best_seg = seg;
        }
    }

    if (best_cost == std::numeric_limits<double>::infinity()) {
        return {}; // Return empty vector if no valid path exists
    }

    // 3. Generate Local Course
    std::vector<double> lp_x = {0.0};
    std::vector<double> lp_y = {0.0};
    std::vector<double> lp_yaw = {0.0};

    for (int i = 0; i < 3; i++) {
        if (best_seg.len[i] == 0.0) continue;

        double origin_x = lp_x.back();
        double origin_y = lp_y.back();
        double origin_yaw = lp_yaw.back();

        double current_length = step_size;
        while (std::abs(current_length + step_size) <= std::abs(best_seg.len[i])) {
            interpolate(current_length, best_seg.mode[i], curvature, origin_x, origin_y, origin_yaw, lp_x, lp_y, lp_yaw);
            current_length += step_size;
        }
        
        interpolate(best_seg.len[i], best_seg.mode[i], curvature, origin_x, origin_y, origin_yaw, lp_x, lp_y, lp_yaw);
    }

    // 4. Convert local trajectory back to the global coordinate frame
    std::vector<lemlib::Pose> raw_poses;
    raw_poses.reserve(lp_x.size());
    
    for (size_t i = 0; i < lp_x.size(); i++) {
        double g_x_path = std::cos(s_yaw) * lp_x[i] - std::sin(s_yaw) * lp_y[i] + s_x;
        double g_y_path = std::sin(s_yaw) * lp_x[i] + std::cos(s_yaw) * lp_y[i] + s_y;
        double g_yaw_path = angle_mod(lp_yaw[i] + s_yaw);
        raw_poses.push_back(lemlib::Pose(g_x_path, g_y_path, g_yaw_path));
    }

    // 5. Compute "Distance to End" for all points efficiently (O(N) backwards pass)
    std::vector<float> dist_to_end(raw_poses.size(), 0.0f);
    for (int i = raw_poses.size() - 2; i >= 0; i--) {
        dist_to_end[i] = dist_to_end[i+1] + raw_poses[i].distance(raw_poses[i+1]);
    }

    // 6. Compute Velocity and Time Profile
    std::vector<DubinsMath::PathPoint> final_path;
    final_path.reserve(raw_poses.size());
    float current_time = 0.0f;
    
    // Constant for converting VEX velocity (0-127) to real-world Inches Per Second (IPS)
    // You may need to tune this constant (e.g., 45.0) to match your physical robot's top speed!
    const float MAX_ROBOT_IPS = 63.0f; 

    for (int i = 0; i < raw_poses.size(); i++) {
        float target_v = maxSpeed;

        // A. Curvature Slowdown
        if (i > 0 && i < raw_poses.size() - 1) {
            float dist = raw_poses[i-1].distance(raw_poses[i+1]);
            // Calculate change in angle over distance
            float dTheta = angle_mod(raw_poses[i+1].theta - raw_poses[i-1].theta);
            float local_curv = (dist > 0.001f) ? std::abs(dTheta) / dist : 0.0f;

            if (local_curv > 0.01f) {
                target_v = std::min(target_v, 127.0f / (1.0f + local_curv * horizontalDrift));
            }
        }

        // B. Distance-to-End Deceleration (The "Brake")
        float brakeDistance = slowDist; 
        if (dist_to_end[i] < brakeDistance) {
            target_v = std::min(target_v, (dist_to_end[i] / brakeDistance) * maxSpeed);
        }

        // C. Apply Minimum Speed limits
        if (target_v < minSpeed && i != raw_poses.size() - 1) {
            target_v = minSpeed;
        }

        // D. Ensure the final point is 0 velocity to trigger stops
        if (i == raw_poses.size() - 1) {
            target_v = 0.0f;
        }

        // E. Integrate Time (dt = distance / velocity)
        if (i > 0) {
            float step_dist = raw_poses[i-1].distance(raw_poses[i]);
            // Convert target_v (0-127) to real world Inches Per Second.
            // std::max prevents division by zero if target_v drops very low
            float v_ips = std::max(target_v * (MAX_ROBOT_IPS / 127.0f), 0.5f); 
            current_time += step_dist / v_ips;
        }

        final_path.push_back({raw_poses[i], target_v, current_time});
    }

    return final_path;
}

/**
 * @brief find the closest point on the path to the robot within a safe forward window
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @param lastIndex the closest point found in the previous loop iteration
 * @param searchWindow how many points ahead to allow searching (default is 10)
 * @return int index to the closest point
 */
int findClosestNew(lemlib::Pose pose, const std::vector<lemlib::Pose>& path, int lastIndex, int searchWindow = 10) {
    int closestPoint = lastIndex;
    float closestDist = std::numeric_limits<float>::infinity();

    // Calculate the max index we are allowed to check to prevent skipping turns
    // std::min prevents us from checking past the end of the path array
    int endIndex = std::min((int)path.size(), lastIndex + searchWindow);

    // Only loop through points from where we currently are, up to the search limit
    for (int i = lastIndex; i < endIndex; i++) {
        const float dist = pose.distance(path.at(i));
        if (dist < closestDist) { // new closest point
            closestDist = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

void lemlib::Chassis::ramseteToPose(float x, float y, float theta, int timeout, RamseteToPoseParams params, bool async)
{
    this->requestMotionStart();
    if (!this->motionRunning) return;
    
    if (async) {
        pros::Task task([&]() { ramseteToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    lemlib::Pose target(x, y, theta);
    lemlib::Pose startPos = getPose();

    if(params.turningRadius == 0) params.turningRadius = drivetrain.trackWidth * 0.75;
    float pathCurvature = 1.00f / params.turningRadius;
    if(params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // --- Generate the path with velocities and times embedded ---
    
    // LemLib's getPose() without parameters returns DEGREES. 
    // Convert both the starting heading and target heading to Standard Math Radians
    float math_start_yaw = (90.0f - startPos.theta) * (M_PI / 180.0f);
    float math_end_yaw = (90.0f - target.theta) * (M_PI / 180.0f);

    std::vector<DubinsMath::PathPoint> pathPoints = generate_dubins_path(
        startPos.x, startPos.y, math_start_yaw, 
        target.x, target.y, math_end_yaw, 
        pathCurvature, params.resolution, 
        params.minSpeed, params.maxSpeed, params.horizontalDrift, params.slowdownRange
    );

    if (pathPoints.size() == 0) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    if(params.outputDebug) {
        for (const auto& pt : pathPoints) {
            std::cout << pt.pose.x << ", " << pt.pose.y << ", " << pt.pose.theta << " | v: " << pt.v << " t: " << pt.t << std::endl;
        }
    }

    // --- State Variables ---
    Pose pose = this->getPose(true);
    int targetIndex = 0; // Replaces closestPoint
    int compState = pros::competition::get_status();
    distTraveled = 0;

    Timer timer(timeout);

    // Start a timer right before the loop begins
    uint32_t startTime = pros::millis();

    bool ranEarlyLambda = false;

    // --- Ramsete Tracking Loop ---
    while (!timer.isDone() && this->motionRunning) {
        pose = this->getPose(true);
        if (!params.forwards) pose.theta -= M_PI;

        // 1. Distance-Based Index Searching
        // We search up to 15 points ahead of our current index to find the physically closest point.
        // This guarantees the target NEVER outruns the robot, no matter how slow the robot drives!
        int searchEnd = std::min((int)pathPoints.size(), targetIndex + 15);
        float minDist = std::numeric_limits<float>::infinity();
        
        for (int j = targetIndex; j < searchEnd; j++) {
            float dist = pose.distance(pathPoints[j].pose);
            if (dist < minDist) {
                minDist = dist;
                targetIndex = j; // Lock the target to the robot's physical location
            }
        }

        // 2. Break Conditions
        // Break if we reach the end of the array, or physically get within 3 inches of the goal
        if (pose.distance(pathPoints.back().pose) < params.pidExitRange) {
            break;
        }

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        if (distTarget <= params.earlyLambdaRange && params.earlyLambda != nullptr && !ranEarlyLambda)
        {
            new pros::Task(params.earlyLambda);
            ranEarlyLambda = true;
        }

        // 3. Extract Target Data
        lemlib::Pose targetPose = pathPoints.at(targetIndex).pose;
        float v_d_voltage = pathPoints.at(targetIndex).v; 

        // --- THE UNIT CONVERSION FIX ---
        // Ramsete MUST use physical units. Convert 0-127 to Inches Per Second.
        // Change 45.0 to match your robot's actual top speed in in/s!
        const float MAX_IPS = 63.0f; 
        float v_d = v_d_voltage * (MAX_IPS / 127.0f); 

        // 4. Calculate Desired Angular Velocity (w_d) in Radians/Sec
        float w_d = 0;
        if (targetIndex < pathPoints.size() - 1) {
            float dTheta = DubinsMath::angle_mod(pathPoints.at(targetIndex+1).pose.theta - targetPose.theta);
            float dt = pathPoints.at(targetIndex+1).t - pathPoints.at(targetIndex).t;
            if (dt > 0.001f) {
                w_d = dTheta / dt;
            }
        }

        // --- Coordinate Conversions ---
        float theta_desired = targetPose.theta; 
        float theta_actual = (M_PI / 2.0) - pose.theta; 

        // 5. Compute Error (in INCHES and RADIANS)
        float dX = targetPose.x - pose.x;
        float dY = targetPose.y - pose.y;
        
        float eX = cos(theta_actual) * dX + sin(theta_actual) * dY;
        float eY = -sin(theta_actual) * dX + cos(theta_actual) * dY;
        float eTheta = DubinsMath::angle_mod(theta_desired - theta_actual);

        // 6. Compute Ramsete Gain
        // IMPORTANT: Because our error is in Inches, not Meters, b must be scaled down!
        // Standard meter b=2.0 becomes b=0.0013 in inches ( 2.0 / 39.37^2 ).
        float b = params.b; // You can still use params.b here if you update it in your function call!
        float zeta = params.zeta; // 0.7 is still perfectly fine
        float k = 2 * zeta * sqrt(pow(w_d, 2) + b * pow(v_d, 2));

        // 7. Compute Output Velocities (in INCHES PER SECOND)
        float v = v_d * cos(eTheta) + k * eX;
        float sinc = (std::abs(eTheta) < 1e-4) ? 1.0 : sin(eTheta) / eTheta;
        float w = w_d + k * eTheta + (b * v_d * sinc * eY);

        // 8. Inverse Kinematics (in INCHES PER SECOND)
        float angularMotorVelocity = w * drivetrain.trackWidth / 2.0;
        float left_ips = v - angularMotorVelocity;
        float right_ips = v + angularMotorVelocity;

        // --- CONVERT BACK TO VOLTAGE (0-127) ---
        float targetLeftVel = left_ips * (127.0f / MAX_IPS);
        float targetRightVel = right_ips * (127.0f / MAX_IPS);

        // Desaturation (Prevents ratio breaking at high speeds)
        float max_mag = std::max(std::abs(targetLeftVel), std::abs(targetRightVel));
        if (max_mag > 127.0) {
            targetLeftVel = (targetLeftVel / max_mag) * 127.0;
            targetRightVel = (targetRightVel / max_mag) * 127.0;
        }

        // Final Motor Output
        if (params.forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }

    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);

    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    distTraveled = 0;
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning && params.pidExitRange != 0) {
        // update variables
        Pose pose = getPose();

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));

        targetTheta = theta;

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, AngularDirection::AUTO);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        drivetrain.leftMotors->move(motorPower);
        drivetrain.rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}