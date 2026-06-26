#pragma once

#include "pros/distance.hpp"

#include <optional>
#include <string>
#include <vector>

namespace wallsnap {

enum class CoordinateMode {
    ABSOLUTE_FIELD,
    START_RELATIVE,
};

enum class FieldOrigin {
    BOTTOM_LEFT,
    FIELD_CENTER,
};

enum class Wall {
    LEFT,
    RIGHT,
    FRONT,
    BACK,
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct DistanceReading {
    int distanceMM = 0;
    int confidence = 0;
    bool valid = false;
};

struct SensorConfig {
    pros::Distance* sensor = nullptr;
    double offsetX = 0.0;
    double offsetY = 0.0;
    double angleOffsetDeg = 0.0;
    Wall targetWall = Wall::FRONT;
    std::string name;
};

struct FieldConfig {
    double fieldWidth = 144.0;
    double fieldHeight = 144.0;
    FieldOrigin origin = FieldOrigin::BOTTOM_LEFT;
};

struct WallSnapConfig {
    FieldConfig field;
    CoordinateMode coordinateMode = CoordinateMode::ABSOLUTE_FIELD;
    Pose startAbsolutePose;

    int samples = 7;
    int minDistanceMM = 80;
    int maxDistanceMM = 1500;
    int minConfidence = 35;
    double maxAngleErrorDeg = 12.0;
    double maxDistanceErrorInches = 12.0;
    double blend = 1.0;
};

struct ResetResult {
    bool success = false;

    double correctedX = 0.0;
    double correctedY = 0.0;

    bool correctedXValid = false;
    bool correctedYValid = false;

    double correctedAbsoluteX = 0.0;
    double correctedAbsoluteY = 0.0;

    bool correctedAbsoluteXValid = false;
    bool correctedAbsoluteYValid = false;

    std::string message;
    std::vector<std::string> debugLog;
};

class WallSnap {
public:
    explicit WallSnap(const WallSnapConfig& config);

    void addSensor(const SensorConfig& sensor);

    ResetResult calculateCorrection(const Pose& currentPose);
    ResetResult calculateHardCorrection(const Pose& currentPose);
    ResetResult calculateSoftCorrection(const Pose& currentPose, double blend);

private:
    struct AxisCandidates {
        std::vector<double> x;
        std::vector<double> y;
    };

    WallSnapConfig config;
    std::vector<SensorConfig> sensors;

    static double mmToInches(double mm);
    static double degToRad(double deg);
    static double normalizeAngleDeg(double angleDeg);
    static double clamp(double value, double minValue, double maxValue);
    static double average(const std::vector<double>& values);

    bool isValidReading(const DistanceReading& reading) const;
    std::string readingRejectReason(const DistanceReading& reading) const;
    std::optional<DistanceReading> getMedianReading(
        const SensorConfig& sensor,
        std::vector<std::string>& debugLog) const;

    Pose toAbsolutePose(const Pose& pose) const;
    Pose fromAbsolutePose(const Pose& absolutePose) const;

    double getWallPosition(Wall wall) const;
    double targetWallDirectionDeg(Wall wall) const;
    double sensorWorldDirectionDeg(const Pose& absolutePose, const SensorConfig& sensor) const;
    bool isSensorFacingWall(const Pose& absolutePose, const SensorConfig& sensor) const;

    double sensorAbsoluteX(const Pose& absolutePose, const SensorConfig& sensor) const;
    double sensorAbsoluteY(const Pose& absolutePose, const SensorConfig& sensor) const;
    std::optional<double> expectedDistanceToWall(
        const Pose& absolutePose,
        const SensorConfig& sensor) const;

    std::optional<double> calculateXFromSensor(
        const Pose& absolutePose,
        const SensorConfig& sensor,
        double correctedDistanceInches) const;

    std::optional<double> calculateYFromSensor(
        const Pose& absolutePose,
        const SensorConfig& sensor,
        double correctedDistanceInches) const;

    AxisCandidates collectCandidates(const Pose& absolutePose, ResetResult& result) const;
};

}  // namespace wallsnap
