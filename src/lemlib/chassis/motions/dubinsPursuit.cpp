#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"



constexpr double PI = 3.14159265358979323846;

// ------------------- Math Helpers -------------------

double mod2pi(double x) {
    double v = fmod(x, 2.0 * PI);
    if (v < 0) v += 2.0 * PI;
    return v;
}

double angleMod(double x) {
    return mod2pi(x + PI) - PI;
}

// ------------------- Path Types -------------------

bool LSL(double a, double b, double d, double& d1, double& d2, double& d3) {
    double p2 = 2 + d*d - 2*cos(a-b) + 2*d*(sin(a)-sin(b));
    if (p2 < 0) return false;

    double tmp = atan2(cos(b)-cos(a), d + sin(a) - sin(b));
    d1 = mod2pi(-a + tmp);
    d2 = sqrt(p2);
    d3 = mod2pi(b - tmp);
    return true;
}

bool RSR(double a, double b, double d, double& d1, double& d2, double& d3) {
    double p2 = 2 + d*d - 2*cos(a-b) + 2*d*(sin(b)-sin(a));
    if (p2 < 0) return false;

    double tmp = atan2(cos(a)-cos(b), d - sin(a) + sin(b));
    d1 = mod2pi(a - tmp);
    d2 = sqrt(p2);
    d3 = mod2pi(-b + tmp);
    return true;
}

bool LSR(double a, double b, double d, double& d1, double& d2, double& d3) {
    double p2 = -2 + d*d + 2*cos(a-b) + 2*d*(sin(a)+sin(b));
    if (p2 < 0) return false;

    d2 = sqrt(p2);
    double tmp = atan2(-cos(a)-cos(b), d + sin(a) + sin(b) - atan2(-2.0, d2));
    d1 = mod2pi(-a + tmp);
    d3 = mod2pi(-b + tmp);
    return true;
}

bool RSL(double a, double b, double d, double& d1, double& d2, double& d3) {
    double p2 = d*d - 2 + 2*cos(a-b) - 2*d*(sin(a)+sin(b));
    if (p2 < 0) return false;

    d2 = sqrt(p2);
    double tmp = atan2(cos(a)+cos(b), d - sin(a) - sin(b)) - atan2(2.0, d2);
    d1 = mod2pi(a - tmp);
    d3 = mod2pi(b - tmp);
    return true;
}

bool RLR(double a, double b, double d, double& d1, double& d2, double& d3) {
    double tmp = (6 - d*d + 2*cos(a-b) + 2*d*(sin(a)-sin(b))) / 8.0;
    if (fabs(tmp) > 1) return false;

    d2 = mod2pi(2*PI - acos(tmp));
    d1 = mod2pi(a - atan2(cos(a)-cos(b), d - sin(a)+sin(b)) + d2/2);
    d3 = mod2pi(a - b - d1 + d2);
    return true;
}

bool LRL(double a, double b, double d, double& d1, double& d2, double& d3) {
    double tmp = (6 - d*d + 2*cos(a-b) + 2*d*(-sin(a)+sin(b))) / 8.0;
    if (fabs(tmp) > 1) return false;

    d2 = mod2pi(2*PI - acos(tmp));
    d1 = mod2pi(-a - atan2(cos(a)-cos(b), d + sin(a)-sin(b)) + d2/2);
    d3 = mod2pi(b - a - d1 + d2);
    return true;
}

// ------------------- Interpolation -------------------

void interpolate(double length, char mode, double k,
                 double ox, double oy, double oyaw,
                 std::vector<double>& px,
                 std::vector<double>& py,
                 std::vector<double>& pyaw) {

    if (mode == 'S') {
        px.push_back(ox + length / k * cos(oyaw));
        py.push_back(oy + length / k * sin(oyaw));
        pyaw.push_back(oyaw);
    } else {
        double ldx = sin(length) / k;
        double ldy = (mode == 'L') ?
            (1 - cos(length)) / k :
            (1 - cos(length)) / -k;

        double gdx = cos(-oyaw)*ldx + sin(-oyaw)*ldy;
        double gdy = -sin(-oyaw)*ldx + cos(-oyaw)*ldy;

        px.push_back(ox + gdx);
        py.push_back(oy + gdy);

        pyaw.push_back(mode == 'L' ? oyaw + length : oyaw - length);
    }
}

void generateCourse(const std::vector<double>& lengths,
                    const std::vector<char>& modes,
                    double k, double step,
                    std::vector<double>& px,
                    std::vector<double>& py,
                    std::vector<double>& pyaw) {

    px = {0}; py = {0}; pyaw = {0};

    for (int i = 0; i < modes.size(); i++) {
        double len = lengths[i];
        if (len == 0) continue;

        double ox = px.back();
        double oy = py.back();
        double oyaw = pyaw.back();

        double dist = step;
        while (fabs(dist + step) <= fabs(len)) {
            interpolate(dist, modes[i], k, ox, oy, oyaw, px, py, pyaw);
            dist += step;
        }

        interpolate(len, modes[i], k, ox, oy, oyaw, px, py, pyaw);
    }
}

// ------------------- Main Planner -------------------

std::vector<lemlib::Pose> planDubins(double sx, double sy, double syaw, double gx, double gy, double gyaw, double curvature, double step) {

    // Transform to local frame
    double dx = gx - sx;
    double dy = gy - sy;

    double lx = cos(syaw)*dx + sin(syaw)*dy;
    double ly = -sin(syaw)*dx + cos(syaw)*dy;
    double lyaw = gyaw - syaw;

    double D = sqrt(lx*lx + ly*ly);
    double d = D * curvature;

    double theta = mod2pi(atan2(ly, lx));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(lyaw - theta);

    struct Path {
        std::vector<char> mode;
        std::vector<double> lengths;
        double cost;
    };

    std::vector<Path> candidates;

    auto tryPath = [&](auto func, std::vector<char> mode) {
        double d1, d2, d3;
        if (func(alpha, beta, d, d1, d2, d3)) {
            double cost = fabs(d1) + fabs(d2) + fabs(d3);
            candidates.push_back({mode, {d1, d2, d3}, cost});
        }
    };

    tryPath(LSL, {'L','S','L'});
    tryPath(RSR, {'R','S','R'});
    tryPath(LSR, {'L','S','R'});
    tryPath(RSL, {'R','S','L'});
    tryPath(RLR, {'R','L','R'});
    tryPath(LRL, {'L','R','L'});

    if (candidates.empty()) return {};

    // Choose shortest
    Path best = candidates[0];
    for (auto& p : candidates) {
        if (p.cost < best.cost) best = p;
    }

    // Generate path
    std::vector<double> px, py, pyaw;
    generateCourse(best.lengths, best.mode, curvature, step, px, py, pyaw);

    // Convert back to global → Pose
    std::vector<lemlib::Pose> result;

    for (size_t i = 0; i < px.size(); i++) {
        double gx2 = cos(-syaw)*px[i] + sin(-syaw)*py[i] + sx;
        double gy2 = -sin(-syaw)*px[i] + cos(-syaw)*py[i] + sy;
        double gyaw2 = angleMod(pyaw[i] + syaw);

        result.emplace_back(
            gx2,
            gy2,
            gyaw2 * 180.0 / PI
        );
    }

    return result;
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

    theta += 180.0f;
    lemlib::Pose target(x, y, theta);


    if(params.turningRadius == 0) params.turningRadius = drivetrain.trackWidth * 1.5;

    float pathCurvature =  1.00f / params.turningRadius;

    // Use default horizontial drift from the drivetrain class
    if(params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    lemlib::Pose startPos = getPose();

    std::vector<lemlib::Pose> pathPoints = planDubins(startPos.x, startPos.y, atan2(target.y - startPos.y, target.x - startPos.x), target.x, target.y, ((90.0 - target.theta) * PI / 180.0), pathCurvature, params.resolution); // get list of path points
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
        pathPoints[i].theta = degToRad(pathPoints[i].theta);
        path_points_r.push_back(std::pair<lemlib::Pose, float>(pathPoints.at(i), target));
    }

    // 2. Ensure the very last point is exactly 0 to trigger the loop break
    path_points_r.back().second = 0;

    if(params.outputDebug)
    {
        std::ofstream pathDebugOutput("PathingDebug.txt", std::ios::app);
        for (int i = 0; i < path_points_r.size(); i++)
        {
            pathDebugOutput << path_points_r[i].first.x << ", " << path_points_r[i].first.y << ", " << path_points_r[i].first.theta << std::endl;
        }
        pathDebugOutput << "\n\n\n";

        std::ofstream pathDebugOutputTwo("PathingDebugTwo.txt", std::ios::app);
        for (int i = 0; i < path_points_r.size(); i++)
        {
            pathDebugOutputTwo << pathPoints[i].x << ", " << pathPoints[i].y << ", " << pathPoints[i].theta << std::endl;
        }
        pathDebugOutputTwo << "\n\n\n";
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
        float v_d = path_points_r.at(closestPoint).second; // Desired linear velocity

        // 4. Calculate Desired Angular Velocity (w_d)
        // w = v * curvature. We calculate local curvature from the path points.
        float k_d = 0;
        if (closestPoint < path_points_r.size() - 1) {
            k_d = findLookaheadCurvature(path_points_r.at(closestPoint).first, 0, path_points_r.at(closestPoint+1).first);
        }
        float w_d = v_d * k_d;

        // 5. Calculate Errors in the Robot's Local Frame
        float dX = targetPose.x - pose.x;
        float dY = targetPose.y - pose.y;
        float eTheta = angleMod(targetPose.theta - (pose.theta * PI / 180.0)); // Ensure radians

        float eX = cos(pose.theta * PI / 180.0) * dX + sin(pose.theta * PI / 180.0) * dY;
        float eY = -sin(pose.theta * PI / 180.0) * dX + cos(pose.theta * PI / 180.0) * dY;

        // 6. Ramsete Gain Calculation
        // Standard gains: b = 2.0, zeta = 0.7
        float b = params.b; 
        float zeta = params.zeta;
        float k = 2 * zeta * sqrt(pow(w_d, 2) + b * pow(v_d, 2));

        // 7. Compute Adjusted Velocities
        float v = v_d * cos(eTheta) + k * eX;
        // Sinc function (sin(x)/x) handles the case where eTheta is near zero
        float sinc = (std::abs(eTheta) < 1e-4) ? 1.0 : sin(eTheta) / eTheta;
        float w = w_d + b * v_d * sinc * eY + k * eTheta;

        // 8. Output to Motors (Inverse Kinematics)
        float targetLeftVel = v + (w * drivetrain.trackWidth / 2);
        float targetRightVel = v - (w * drivetrain.trackWidth / 2);

        pros::delay(10);
    }

    // stop the robot
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    // give the mutex back
    this->endMotion();
}