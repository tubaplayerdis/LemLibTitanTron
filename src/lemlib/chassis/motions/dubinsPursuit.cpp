#include <vector>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"



#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>

// Assuming lemlib::Pose is available in your project environment
// #include "lemlib/api.hpp" 

namespace DubinsMath {

    const double PI = 3.14159265358979323846;

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
 * Generates an optimal Dubins path 
 * * @param s_x Start X position
 * @param s_y Start Y position
 * @param s_yaw Start Yaw angle (radians)
 * @param g_x Goal X position
 * @param g_y Goal Y position
 * @param g_yaw Goal Yaw angle (radians)
 * @param curvature The curvature limit (1.0 / radius)
 * @param step_size The spacing between generated points on the path
 * @return A vector of lemlib::Pose objects representing the global trajectory
 */
std::vector<lemlib::Pose> generate_dubins_path(double s_x, double s_y, double s_yaw, 
                                               double g_x, double g_y, double g_yaw, 
                                               double curvature, double step_size = 0.1) {
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

    // Test all planners mapping directly to the python func map
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
    std::vector<lemlib::Pose> final_path;
    final_path.reserve(lp_x.size());
    
    for (size_t i = 0; i < lp_x.size(); i++) {
        // Rotate local coordinates back by the starting yaw, then translate by start position
        double g_x_path = std::cos(s_yaw) * lp_x[i] - std::sin(s_yaw) * lp_y[i] + s_x;
        double g_y_path = std::sin(s_yaw) * lp_x[i] + std::cos(s_yaw) * lp_y[i] + s_y;
        double g_yaw_path = angle_mod(lp_yaw[i] + s_yaw);
        
        final_path.push_back(lemlib::Pose(g_x_path, g_y_path, g_yaw_path));
    }

    return final_path;
}

void lemlib::Chassis::pursuitToPose(float x, float y, float theta, int timeout, PursuitToPoseParams params, bool async)
{
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { pursuitToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    lemlib::Pose target(x, y, theta);


    if(params.turningRadius == 0) params.turningRadius = drivetrain.trackWidth * 1.5;

    float pathCurvature =  1.00f / params.turningRadius;

    // Use default horizontial drift from the drivetrain class
    if(params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    lemlib::Pose startPos = getPose();

    std::vector<lemlib::Pose> pathPoints = generate_dubins_path(startPos.x, startPos.y, atan2(target.y - startPos.y, target.x - startPos.x), target.x, target.y, degToRad(target.theta - 90.0f), pathCurvature, params.resolution); // get list of path points
    if (pathPoints.size() == 0) {
        // set distTraveled to -1 to indicate that the function has finished
        distTraveled = -1;
        // give the mutex back
        this->endMotion();
        return;
    }

    std::vector<std::pair<lemlib::Pose, float>> path_points_r;

    //replace theta values with power values
    for (int i = 0; i < pathPoints.size(); i++) {
        // Start with the maximum allowed speed
        float target = params.maxSpeed;

        // A. Curvature-Based Velocity (Automatic Slowdown for sharp turns)
        // We calculate the distance between points to find local curvature
        if (i > 0 && i < pathPoints.size() - 1) {
            float curvature = findLookaheadCurvature(pathPoints.at(i-1), 0, pathPoints.at(i+1));
            if (std::abs(curvature) > 0.01) {
                // Formula: velocity = sqrt(max_centripetal_accel / curvature)
                // Simplified: lower speed as curvature increases
                target = std::min(target, 127.0f / (1.0f + std::abs(curvature) * params.horizontalDrift));
            }
        }

        // B. Distance-to-End Deceleration (The "Brake")
        float distToEnd = 0;
        for (int j = i; j < pathPoints.size() - 1; j++) {
            distToEnd += pathPoints.at(j).distance(pathPoints.at(j+1));
        }

        // Linear ramp down: if within 12 inches, start slowing down
        float brakeDistance = 12.0; 
        if (distToEnd < brakeDistance) {
            target = std::min(target, (distToEnd / brakeDistance) * params.maxSpeed + 15);

            if(target < params.minSpeed) target = params.minSpeed;
        }

        if(params.minSpeedOverride && target < params.minSpeed) target = params.minSpeed;
        // Save calculated velocity into theta
        pathPoints[i].theta = DubinsMath::PI/2 - pathPoints[i].theta;
        path_points_r.push_back(std::pair<lemlib::Pose, float>(pathPoints.at(i), target));
    }

    // 2. Ensure the very last point is exactly 0 to trigger the loop break
    path_points_r.back().second = 0;

    if(params.outputDebug)
    {
        for (int i = 0; i < path_points_r.size(); i++)
        {
            std::cout << pathPoints[i].x << ", " << pathPoints[i].y << ", " << pathPoints[i].theta << std::endl;
        }
    }


    Pose pose = this->getPose(true);
    Pose lastPose = pose;
    Pose lookaheadPose(0, 0, 0);
    Pose lastLookahead = pathPoints.at(0);
    lastLookahead.theta = 0;
    float curvature;
    float targetVel;
    float prevLeftVel = 0;
    float prevRightVel = 0;
    int closestPoint;
    float leftInput = 0;
    float rightInput = 0;
    float prevVel = 0;
    int compState = pros::competition::get_status();
    distTraveled = 0;

    bool ranEarlyLambda = false;

    // Replace the Pure Pursuit loop logic with this Ramsete logic
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        // Inside the for loop...
        pose = this->getPose(true);
        if (!params.forwards) pose.theta -= M_PI;

        // 1. Find the closest point in the new pair-based vector
        // Note: You'll need to update findClosest to handle the std::pair structure
        closestPoint = findClosest(pose, pathPoints); 

        // 2. Break if velocity target is 0 (end of path)
        if (path_points_r.at(closestPoint).second == 0) break;

        // 3. Extract Target Data
        lemlib::Pose targetPose = path_points_r.at(closestPoint).first;
        float v_d = path_points_r.at(closestPoint).second;

        // Convert LemLib degrees/radians to Standard Math Radians (0=East, CCW)
        // targetPose.theta is in degrees from the Dubins generator
        float math_target_theta = DubinsMath::PI / 2.0 - targetPose.theta;
        // pose.theta from getPose(true) is in radians
        float math_pose_theta = DubinsMath::PI / 2.0 - pose.theta; 

        // 4. Calculate Desired Angular Velocity (w_d)
        float k_d = 0;
        if (closestPoint < path_points_r.size() - 1) {
            k_d = findLookaheadCurvature(path_points_r.at(closestPoint).first, 0, path_points_r.at(closestPoint+1).first);
        }
        float w_d = v_d * k_d;

        // 5. Calculate Errors in the Robot's Local Frame (Using Math Radians)
        float dX = targetPose.x - pose.x;
        float dY = targetPose.y - pose.y;
        float eTheta = DubinsMath::angle_mod(math_target_theta - math_pose_theta);

        float eX = cos(math_pose_theta) * dX + sin(math_pose_theta) * dY;
        float eY = -sin(math_pose_theta) * dX + cos(math_pose_theta) * dY;

        // 6. Ramsete Gain Calculation
        float b = params.b; 
        float zeta = params.zeta;
        float k = 2 * zeta * sqrt(pow(w_d, 2) + b * pow(v_d, 2));

        // 7. Compute Adjusted Velocities
        float v = v_d * cos(eTheta) + k * eX;
        float sinc = (std::abs(eTheta) < 1e-4) ? 1.0 : sin(eTheta) / eTheta;
        float w = w_d + b * v_d * sinc * eY + k * eTheta;

        // 8. Output to Motors (Inverse Kinematics)
        float targetLeftVel = v - (w * drivetrain.trackWidth / 2);
        float targetRightVel = v + (w * drivetrain.trackWidth / 2);

        if (params.forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

        pros::delay(10);
    }

    std::cout << "Done!" << std::endl;

    // stop the robot
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    // give the mutex back
    this->endMotion();
}