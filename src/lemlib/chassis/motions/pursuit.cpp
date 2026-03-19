// The implementation below is mostly based off of
// the document written by Dawgma
// Here is a link to the original document
// https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf

#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"

// Structure to hold segment lengths and metadata
struct DubinsPath {
    float L1, L2, L3; // Lengths of segments
    float c1x, c1y, c2x, c2y; // Centers of the two circles
    int type; // 0: LSL, 1: RSR, 2: LSR, 3: RSL
    float totalDist;
};

float coterm(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float norm(float a) {
    float r = fmod(a, 2 * M_PI);
    return (r < 0) ? r + 2 * M_PI : r;
}

float dist(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

std::vector<DubinsPath> generateCanidates(float SLx, float SLy, float SRx, float SRy, float GLx, float GLy, float GRx, float GRy, float rho, float startAng, float endAng)
{
    std::vector<DubinsPath> candidates;

    // --- LSL Calculation ---
    float dLSL = dist(SLx, SLy, GLx, GLy);
    float thLSL = atan2(GLy - SLy, GLx - SLx);
    float l1_lsl = rho * norm(thLSL - M_PI/2 - startAng);
    float l3_lsl = rho * norm(endAng - (thLSL - M_PI/2));
    candidates.push_back({l1_lsl, dLSL, l3_lsl, SLx, SLy, GLx, GLy, 0, l1_lsl + dLSL + l3_lsl});

    // --- RSR Calculation ---
    float dRSR = dist(SRx, SRy, GRx, GRy);
    float thRSR = atan2(GRy - SRy, GRx - SRx);
    float l1_rsr = rho * norm(startAng - (thRSR + M_PI/2));
    float l3_rsr = rho * norm((thRSR + M_PI/2) - endAng);
    candidates.push_back({l1_rsr, dRSR, l3_rsr, SRx, SRy, GRx, GRy, 1, l1_rsr + dRSR + l3_rsr});

    // --- LSR Calculation (Check if d > 2*rho) ---
    float dLSR = dist(SLx, SLy, GRx, GRy);
    if (dLSR > 2 * rho) {
        float thLSR = atan2(GRy - SLy, GRx - SLx) + acos(2 * rho / dLSR);
        float l1_lsr = rho * norm(thLSR - startAng);
        float l3_lsr = rho * norm(thLSR - M_PI - endAng);
        float mid_lsr = sqrt(dLSR * dLSR - 4 * rho * rho);
        candidates.push_back({l1_lsr, mid_lsr, l3_lsr, SLx, SLy, GRx, GRy, 2, l1_lsr + mid_lsr + l3_lsr});
    }

    // --- RSL Calculation (Check if d > 2*rho) ---
    float dRSL = dist(SRx, SRy, GLx, GLy);
    if (dRSL > 2 * rho) {
        float thRSL = atan2(GLy - SRy, GLx - SRx) - acos(2 * rho / dRSL);
        float l1_rsl = rho * norm(startAng - thRSL);
        float l3_rsl = rho * norm(endAng - (thRSL + M_PI));
        float mid_rsl = sqrt(dRSL * dRSL - 4 * rho * rho);
        candidates.push_back({l1_rsl, mid_rsl, l3_rsl, SRx, SRy, GLx, GLy, 3, l1_rsl + mid_rsl + l3_rsl});
    }

    return candidates;
}

std::vector<lemlib::Pose> generatePath(float resolution, float rho, float startAng, float endAng, DubinsPath best, lemlib::Pose start, lemlib::Pose end)
{
    std::vector<lemlib::Pose> path;
    float res = resolution; // 2-inch point spacing

    path.push_back(start);

    // --- SEGMENT 1: Initial Arc ---
    // Determine direction: Left (+1) or Right (-1)
    float dir1 = (best.type == 0 || best.type == 2) ? 1.0 : -1.0;
    float startAngle1 = startAng + (dir1 * M_PI/2); 
    
    for (float s = 0; s < best.L1; s += res) {
        float currentAngle = startAngle1 + (dir1 * s / rho);
        float x = best.c1x + rho * cos(currentAngle);
        float y = best.c1y + rho * sin(currentAngle);
        // Heading is tangent to the circle
        float heading = currentAngle + (dir1 * M_PI/2); 
        path.push_back(lemlib::Pose(x, y, lemlib::radToDeg(M_PI/2 - heading)));
    }

    // --- SEGMENT 2: Straight Tangent ---
    // Get the last point of Segment 1 to start the straight
    lemlib::Pose t1 = path.back();
    float dx = cos(lemlib::degToRad(90 - t1.theta));
    float dy = sin(lemlib::degToRad(90 - t1.theta));

    for (float s = 0; s < best.L2; s += res) {
        path.push_back(lemlib::Pose(t1.x + dx * s, t1.y + dy * s, t1.theta));
    }

    // --- SEGMENT 3: Final Arc ---
    float dir3 = (best.type == 0 || best.type == 3) ? 1.0 : -1.0;
    lemlib::Pose t2 = path.back();
    float startAngle3 = lemlib::degToRad(90 - t2.theta) - (dir3 * M_PI/2);

    for (float s = 0; s < best.L3; s += res) {
        float currentAngle = startAngle3 + (dir3 * s / rho);
        float x = best.c2x + rho * cos(currentAngle);
        float y = best.c2y + rho * sin(currentAngle);
        float heading = currentAngle + (dir3 * M_PI/2);
        path.push_back(lemlib::Pose(x, y, lemlib::radToDeg(M_PI/2 - heading)));
    }

    path.push_back(end);

    return path;
}

std::vector<lemlib::Pose> generateDubinsPath(lemlib::Pose start, lemlib::Pose end, float resolution,  float rho)
{
    float st = coterm(lemlib::degToRad(start.theta));
    float et = coterm(lemlib::degToRad(end.theta));

    // 2. Define all 4 Circle Centers
    float SLx = start.x + rho * cos(st + M_PI/2), SLy = start.y + rho * sin(st + M_PI/2);
    float SRx = start.x + rho * cos(st - M_PI/2), SRy = start.y + rho * sin(st - M_PI/2);
    float GLx = end.x + rho * cos(et + M_PI/2), GLy = end.y + rho * sin(et + M_PI/2);
    float GRx = end.x + rho * cos(et - M_PI/2), GRy = end.y + rho * sin(et - M_PI/2);

    std::vector<DubinsPath> candidates = generateCanidates(SLx, SLy, SRx, SRy, GLx, GLy, GRx, GRy, rho, st, et);

    // 3. Find the Best Path
    DubinsPath best = *std::min_element(candidates.begin(), candidates.end(), [](const DubinsPath& a, const DubinsPath& b) {
        return a.totalDist < b.totalDist;
    });

    std::vector<lemlib::Pose> path = generatePath(resolution, rho, st, et, best, start, end);
    
    return path;
}

/**
 * @brief function that returns elements in a file line, separated by a delimeter
 *
 * @param input the raw string
 * @param delimeter string separating the elements in the line
 * @return std::vector<std::string> array of elements read from the file
 */
std::vector<std::string> readElement(const std::string& input, const std::string& delimiter) {
    std::string token;
    std::string s = input;
    std::vector<std::string> output;
    size_t pos = 0;

    // main loop
    while ((pos = s.find(delimiter)) != std::string::npos) { // while there are still delimiters in the string
        token = s.substr(0, pos); // processed substring
        output.push_back(token);
        s.erase(0, pos + delimiter.length()); // remove the read substring
    }

    output.push_back(s); // add the last element to the returned string

    return output;
}

/**
 * @brief Convert a string to hex
 *
 * @param input the string to convert
 * @return std::string hexadecimal output
 */
std::string stringToHex(const std::string& input) {
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input) {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}

/**
 * @brief Get a path from the sd card
 *
 * @param filePath The file to read from
 * @return std::vector<lemlib::Pose> vector of points on the path
 */
std::vector<lemlib::Pose> getData(const asset& path) {
    std::vector<lemlib::Pose> robotPath;

    // format data from the asset
    const std::string data(reinterpret_cast<char*>(path.buf), path.size);
    const std::vector<std::string> dataLines = readElement(data, "\n");

    // read the points until 'endData' is read
    for (std::string line : dataLines) {
        lemlib::infoSink()->debug("read raw line {}", stringToHex(line));
        if (line == "endData" || line == "endData\r") break;
        const std::vector<std::string> pointInput = readElement(line, ", "); // parse line
        // check if the line was read correctly
        if (pointInput.size() != 3) {
            lemlib::infoSink()->error("Failed to read path file! Are you using the right format? Raw line: {}",
                                      stringToHex(line));
            break;
        }
        lemlib::Pose pathPoint(0, 0);
        pathPoint.x = std::stof(pointInput.at(0)); // x position
        pathPoint.y = std::stof(pointInput.at(1)); // y position
        pathPoint.theta = std::stof(pointInput.at(2)); // velocity
        robotPath.push_back(pathPoint); // save data
        lemlib::infoSink()->debug("read point {}", pathPoint);
    }

    return robotPath;
}

/**
 * @brief find the closest point on the path to the robot
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @return int index to the closest point
 */
int findClosest(lemlib::Pose pose, std::vector<lemlib::Pose> path) {
    int closestPoint;
    float closestDist = infinity();

    // loop through all path points
    for (int i = 0; i < path.size(); i++) {
        const float dist = pose.distance(path.at(i));
        if (dist < closestDist) { // new closest point
            closestDist = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

/**
 * @brief Function that finds the intersection point between a circle and a line
 *
 * @param p1 start point of the line
 * @param p2 end point of the line
 * @param pos position of the robot
 * @param path the path to follow
 * @return float how far along the line the
 */
float circleIntersect(lemlib::Pose p1, lemlib::Pose p2, lemlib::Pose pose, float lookaheadDist) {
    // calculations
    // uses the quadratic formula to calculate intersection points
    lemlib::Pose d = p2 - p1;
    lemlib::Pose f = p1 - pose;
    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookaheadDist * lookaheadDist;
    float discriminant = b * b - 4 * a * c;

    // if a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        // prioritize further down the path
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found
    return -1;
}

/**
 * @brief returns the lookahead point
 *
 * @param lastLookahead - the last lookahead point
 * @param pose - the current position of the robot
 * @param path - the path to follow
 * @param closest - the index of the point closest to the robot
 * @param lookaheadDist - the lookahead distance of the algorithm
 */
lemlib::Pose lookaheadPoint(lemlib::Pose lastLookahead, lemlib::Pose pose, std::vector<lemlib::Pose> path, int closest,
                            float lookaheadDist) {
    // optimizations applied:
    // only consider intersections that have an index greater than or equal to the point closest
    // to the robot
    // and intersections that have an index greater than or equal to the index of the last
    // lookahead point
    const int start = std::max(closest, int(lastLookahead.theta));
    for (int i = start; i < path.size() - 1; i++) {
        lemlib::Pose lastPathPose = path.at(i);
        lemlib::Pose currentPathPose = path.at(i + 1);

        float t = circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

        if (t != -1) {
            lemlib::Pose lookahead = lastPathPose.lerp(currentPathPose, t);
            lookahead.theta = i;
            return lookahead;
        }
    }

    // robot deviated from path, use last lookahead point
    return lastLookahead;
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the lookahead point
 *
 * @param pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return float curvature
 */
float findLookaheadCurvature(lemlib::Pose pose, float heading, lemlib::Pose lookahead) {
    // calculate whether the robot is on the left or right side of the circle
    float side = lemlib::sgn(std::sin(heading) * (lookahead.x - pose.x) - std::cos(heading) * (lookahead.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(heading);
    float c = std::tan(heading) * pose.x - pose.y;
    float x = std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

void lemlib::Chassis::follow(std::vector<lemlib::Pose> path, float lookahead, int timeout, bool forwards, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { follow(path, lookahead, timeout, forwards, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    std::vector<lemlib::Pose> pathPoints = path; // get list of path points
    if (pathPoints.size() == 0) {
        infoSink()->error("No points in path! Do you have the right format? Skipping motion");
        // set distTraveled to -1 to indicate that the function has finished
        distTraveled = -1;
        // give the mutex back
        this->endMotion();
        return;
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

    // loop until the robot is within the end tolerance
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        // get the current position of the robot
        pose = this->getPose(true);
        if (!forwards) pose.theta -= M_PI;

        // update completion vars
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // find the closest point on the path to the robot
        closestPoint = findClosest(pose, pathPoints);
        // if the robot is at the end of the path, then stop
        if (pathPoints.at(closestPoint).theta == 0) break;

        // find the lookahead point
        lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, closestPoint, lookahead);
        lastLookahead = lookaheadPose; // update last lookahead position

        // get the curvature of the arc between the robot and the lookahead point
        float curvatureHeading = M_PI / 2 - pose.theta;
        curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

        // get the target velocity of the robot
        targetVel = pathPoints.at(closestPoint).theta;
        targetVel = slew(targetVel, prevVel, lateralSettings.slew);
        prevVel = targetVel;

        // calculate target left and right velocities
        float targetLeftVel = targetVel * (2 + curvature * drivetrain.trackWidth) / 2;
        float targetRightVel = targetVel * (2 - curvature * drivetrain.trackWidth) / 2;

        // ratio the speeds to respect the max speed
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // update previous velocities
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;

        // move the drivetrain
        if (forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

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

void lemlib::Chassis::follow(const asset& path, float lookahead, int timeout, bool forwards, bool async) {
    follow(getData(path), lookahead, timeout, forwards, async);
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

    float turningRadius = drivetrain.trackWidth / 2;

    // Use default horizontial drift from the drivetrain class
    if(params.horizontalDrift = 0) params.horizontalDrift = drivetrain.horizontalDrift;

    std::vector<lemlib::Pose> pathPoints = generateDubinsPath(getPose(), target, params.resolution, turningRadius); // get list of path points
    if (pathPoints.size() == 0) {
        infoSink()->error("No points in path! Do you have the right format? Skipping motion");
        // set distTraveled to -1 to indicate that the function has finished
        distTraveled = -1;
        // give the mutex back
        this->endMotion();
        return;
    }

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
        }

        // Save calculated velocity into theta
        pathPoints.at(i).theta = target;
    }

    // 2. Ensure the very last point is exactly 0 to trigger the loop break
    pathPoints.back().theta = 0;


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

    // loop until the robot is within the end tolerance
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        // get the current position of the robot
        pose = this->getPose(true);
        if (!params.forwards) pose.theta -= M_PI;

        // update completion vars
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // find the closest point on the path to the robot
        closestPoint = findClosest(pose, pathPoints);
        // if the robot is at the end of the path, then stop
        if (pathPoints.at(closestPoint).theta == 0) break;

        // find the lookahead point
        lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, closestPoint, params.lookahead);
        lastLookahead = lookaheadPose; // update last lookahead position

        // get the curvature of the arc between the robot and the lookahead point
        float curvatureHeading = M_PI / 2 - pose.theta;
        curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

        // get the target velocity of the robot
        targetVel = pathPoints.at(closestPoint).theta;
        targetVel = slew(targetVel, prevVel, lateralSettings.slew);
        prevVel = targetVel;

        // calculate target left and right velocities
        float targetLeftVel = targetVel * (2 + curvature * drivetrain.trackWidth) / 2;
        float targetRightVel = targetVel * (2 - curvature * drivetrain.trackWidth) / 2;

        // ratio the speeds to respect the max speed
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        // check whether the robot is close enough to execute the early lambda
        if (distTarget <= params.earlyLambdaRange && params.earlyLambda != nullptr && !ranEarlyLambda)
        {
            new pros::Task(params.earlyLambda);
            ranEarlyLambda = true;
        }

        // update previous velocities
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;

        // move the drivetrain
        if (params.forwards) {
            drivetrain.leftMotors->move(targetLeftVel);
            drivetrain.rightMotors->move(targetRightVel);
        } else {
            drivetrain.leftMotors->move(-targetRightVel);
            drivetrain.rightMotors->move(-targetLeftVel);
        }

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
