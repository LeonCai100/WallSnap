#include "wallsnap/wall_snap.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace wallsnap {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMmPerInch = 25.4;

std::string sensorName(const SensorConfig& sensor) {
    return sensor.name.empty() ? "unnamed sensor" : sensor.name;
}

std::string formatReject(const SensorConfig& sensor, const std::string& reason) {
    return sensorName(sensor) + ": rejected, " + reason;
}

DistanceReading readOnce(const SensorConfig& sensor) {
    if (sensor.sensor == nullptr) {
        return {};
    }

    return {
        static_cast<int>(sensor.sensor->get()),
        static_cast<int>(sensor.sensor->get_confidence()),
        true,
    };
}

}  // namespace

WallSnap::WallSnap(const WallSnapConfig& config) : config(config) {}

void WallSnap::addSensor(const SensorConfig& sensor) {
    sensors.push_back(sensor);
}

ResetResult WallSnap::calculateCorrection(const Pose& currentPose) {
    return calculateSoftCorrection(currentPose, config.blend);
}

ResetResult WallSnap::calculateHardCorrection(const Pose& currentPose) {
    return calculateSoftCorrection(currentPose, 1.0);
}

ResetResult WallSnap::calculateSoftCorrection(const Pose& currentPose, double blend) {
    ResetResult result;
    const Pose absolutePose = toAbsolutePose(currentPose);
    result.correctedX = currentPose.x;
    result.correctedY = currentPose.y;
    result.correctedAbsoluteX = absolutePose.x;
    result.correctedAbsoluteY = absolutePose.y;

    const AxisCandidates candidates = collectCandidates(absolutePose, result);
    const double safeBlend = clamp(blend, 0.0, 1.0);

    Pose correctedAbsolute = absolutePose;
    if (!candidates.x.empty()) {
        const double hardX = average(candidates.x);
        correctedAbsolute.x = absolutePose.x + (hardX - absolutePose.x) * safeBlend;
        result.correctedAbsoluteX = correctedAbsolute.x;
        result.correctedAbsoluteXValid = true;
    }

    if (!candidates.y.empty()) {
        const double hardY = average(candidates.y);
        correctedAbsolute.y = absolutePose.y + (hardY - absolutePose.y) * safeBlend;
        result.correctedAbsoluteY = correctedAbsolute.y;
        result.correctedAbsoluteYValid = true;
    }

    result.success = result.correctedAbsoluteXValid || result.correctedAbsoluteYValid;
    if (!result.success) {
        result.message = "No valid wall correction was available.";
        return result;
    }

    const Pose correctedOutput = fromAbsolutePose(correctedAbsolute);
    if (result.correctedAbsoluteXValid) {
        result.correctedX = correctedOutput.x;
        result.correctedXValid = true;
    }
    if (result.correctedAbsoluteYValid) {
        result.correctedY = correctedOutput.y;
        result.correctedYValid = true;
    }

    result.message = "WallSnap correction calculated.";
    return result;
}

double WallSnap::mmToInches(double mm) {
    return mm / kMmPerInch;
}

double WallSnap::degToRad(double deg) {
    return deg * kPi / 180.0;
}

double WallSnap::normalizeAngleDeg(double angleDeg) {
    while (angleDeg > 180.0) {
        angleDeg -= 360.0;
    }
    while (angleDeg <= -180.0) {
        angleDeg += 360.0;
    }
    return angleDeg;
}

double WallSnap::clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}

double WallSnap::average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    double total = 0.0;
    for (double value : values) {
        total += value;
    }
    return total / static_cast<double>(values.size());
}

bool WallSnap::isValidReading(const DistanceReading& reading) const {
    return readingRejectReason(reading).empty();
}

std::string WallSnap::readingRejectReason(const DistanceReading& reading) const {
    if (!reading.valid) {
        return "distance sample is invalid or no PROS distance sensor is configured";
    }
    if (reading.distanceMM < config.minDistanceMM) {
        return "distance is below minDistanceMM";
    }
    if (reading.distanceMM > config.maxDistanceMM) {
        return "distance is above maxDistanceMM";
    }
    if (reading.confidence < config.minConfidence) {
        return "confidence is below minConfidence";
    }
    return {};
}

std::optional<DistanceReading> WallSnap::getMedianReading(
    const SensorConfig& sensor,
    std::vector<std::string>& debugLog) const {
    const int sampleCount = std::max(1, config.samples);
    std::vector<DistanceReading> validReadings;
    validReadings.reserve(static_cast<std::size_t>(sampleCount));
    int invalidCount = 0;
    int belowMinCount = 0;
    int aboveMaxCount = 0;
    int lowConfidenceCount = 0;

    for (int i = 0; i < sampleCount; ++i) {
        const DistanceReading reading = readOnce(sensor);
        const std::string rejectReason = readingRejectReason(reading);
        if (rejectReason.empty()) {
            validReadings.push_back(reading);
        } else if (!reading.valid) {
            ++invalidCount;
        } else if (reading.distanceMM < config.minDistanceMM) {
            ++belowMinCount;
        } else if (reading.distanceMM > config.maxDistanceMM) {
            ++aboveMaxCount;
        } else {
            ++lowConfidenceCount;
        }
    }

    if (validReadings.empty()) {
        std::ostringstream message;
        message << sensorName(sensor) << ": rejected, no valid samples"
                << " invalid=" << invalidCount
                << " belowMin=" << belowMinCount
                << " aboveMax=" << aboveMaxCount
                << " lowConfidence=" << lowConfidenceCount;
        debugLog.push_back(message.str());
        return std::nullopt;
    }

    if (invalidCount > 0 || belowMinCount > 0 || aboveMaxCount > 0 || lowConfidenceCount > 0) {
        std::ostringstream message;
        message << sensorName(sensor) << ": ignored samples"
                << " invalid=" << invalidCount
                << " belowMin=" << belowMinCount
                << " aboveMax=" << aboveMaxCount
                << " lowConfidence=" << lowConfidenceCount;
        debugLog.push_back(message.str());
    }

    std::sort(validReadings.begin(), validReadings.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.distanceMM < rhs.distanceMM;
    });
    return validReadings[validReadings.size() / 2];
}

Pose WallSnap::toAbsolutePose(const Pose& pose) const {
    if (config.coordinateMode == CoordinateMode::ABSOLUTE_FIELD) {
        return pose;
    }

    const double headingRad = degToRad(config.startAbsolutePose.theta);
    const double cosHeading = std::cos(headingRad);
    const double sinHeading = std::sin(headingRad);

    return {
        config.startAbsolutePose.x + pose.x * cosHeading - pose.y * sinHeading,
        config.startAbsolutePose.y + pose.x * sinHeading + pose.y * cosHeading,
        normalizeAngleDeg(config.startAbsolutePose.theta + pose.theta),
    };
}

Pose WallSnap::fromAbsolutePose(const Pose& absolutePose) const {
    if (config.coordinateMode == CoordinateMode::ABSOLUTE_FIELD) {
        return absolutePose;
    }

    const double headingRad = degToRad(config.startAbsolutePose.theta);
    const double cosHeading = std::cos(headingRad);
    const double sinHeading = std::sin(headingRad);
    const double dx = absolutePose.x - config.startAbsolutePose.x;
    const double dy = absolutePose.y - config.startAbsolutePose.y;

    return {
        dx * cosHeading + dy * sinHeading,
        -dx * sinHeading + dy * cosHeading,
        normalizeAngleDeg(absolutePose.theta - config.startAbsolutePose.theta),
    };
}

double WallSnap::getWallPosition(Wall wall) const {
    const bool centered = config.field.origin == FieldOrigin::FIELD_CENTER;
    switch (wall) {
        case Wall::LEFT:
            return centered ? -config.field.fieldWidth / 2.0 : 0.0;
        case Wall::RIGHT:
            return centered ? config.field.fieldWidth / 2.0 : config.field.fieldWidth;
        case Wall::BACK:
            return centered ? -config.field.fieldHeight / 2.0 : 0.0;
        case Wall::FRONT:
            return centered ? config.field.fieldHeight / 2.0 : config.field.fieldHeight;
    }
    return 0.0;
}

double WallSnap::targetWallDirectionDeg(Wall wall) const {
    switch (wall) {
        case Wall::FRONT:
            return 0.0;
        case Wall::LEFT:
            return 90.0;
        case Wall::RIGHT:
            return -90.0;
        case Wall::BACK:
            return 180.0;
    }
    return 0.0;
}

double WallSnap::sensorWorldDirectionDeg(const Pose& absolutePose, const SensorConfig& sensor) const {
    return normalizeAngleDeg(absolutePose.theta + sensor.angleOffsetDeg);
}

bool WallSnap::isSensorFacingWall(const Pose& absolutePose, const SensorConfig& sensor) const {
    const double error = normalizeAngleDeg(
        sensorWorldDirectionDeg(absolutePose, sensor) - targetWallDirectionDeg(sensor.targetWall));
    return std::abs(error) <= config.maxAngleErrorDeg;
}

double WallSnap::sensorAbsoluteX(const Pose& absolutePose, const SensorConfig& sensor) const {
    const double headingRad = degToRad(absolutePose.theta);
    const double rightX = std::cos(headingRad);
    const double frontX = -std::sin(headingRad);
    return absolutePose.x + sensor.offsetX * rightX + sensor.offsetY * frontX;
}

double WallSnap::sensorAbsoluteY(const Pose& absolutePose, const SensorConfig& sensor) const {
    const double headingRad = degToRad(absolutePose.theta);
    const double rightY = std::sin(headingRad);
    const double frontY = std::cos(headingRad);
    return absolutePose.y + sensor.offsetX * rightY + sensor.offsetY * frontY;
}

std::optional<double> WallSnap::calculateXFromSensor(
    const Pose& absolutePose,
    const SensorConfig& sensor,
    double correctedDistanceInches) const {
    const double sensorX = sensorAbsoluteX(absolutePose, sensor);
    switch (sensor.targetWall) {
        case Wall::LEFT:
            return getWallPosition(Wall::LEFT) + correctedDistanceInches - (sensorX - absolutePose.x);
        case Wall::RIGHT:
            return getWallPosition(Wall::RIGHT) - correctedDistanceInches - (sensorX - absolutePose.x);
        case Wall::FRONT:
        case Wall::BACK:
            return std::nullopt;
    }
    return std::nullopt;
}

std::optional<double> WallSnap::calculateYFromSensor(
    const Pose& absolutePose,
    const SensorConfig& sensor,
    double correctedDistanceInches) const {
    const double sensorY = sensorAbsoluteY(absolutePose, sensor);
    switch (sensor.targetWall) {
        case Wall::BACK:
            return getWallPosition(Wall::BACK) + correctedDistanceInches - (sensorY - absolutePose.y);
        case Wall::FRONT:
            return getWallPosition(Wall::FRONT) - correctedDistanceInches - (sensorY - absolutePose.y);
        case Wall::LEFT:
        case Wall::RIGHT:
            return std::nullopt;
    }
    return std::nullopt;
}

WallSnap::AxisCandidates WallSnap::collectCandidates(
    const Pose& absolutePose,
    ResetResult& result) const {
    AxisCandidates candidates;

    for (const SensorConfig& sensor : sensors) {
        const std::optional<DistanceReading> reading = getMedianReading(sensor, result.debugLog);
        if (!reading.has_value()) {
            continue;
        }

        if (!isSensorFacingWall(absolutePose, sensor)) {
            result.debugLog.push_back(formatReject(sensor, "sensor is not facing its assigned wall"));
            continue;
        }

        const double angleErrorDeg = normalizeAngleDeg(
            sensorWorldDirectionDeg(absolutePose, sensor) - targetWallDirectionDeg(sensor.targetWall));
        const double angleErrorRad = degToRad(angleErrorDeg);
        const double rawDistanceInches = mmToInches(reading->distanceMM);
        const double correctedDistanceInches = rawDistanceInches * std::cos(angleErrorRad);

        std::ostringstream accepted;
        accepted << sensorName(sensor) << ": accepted, median=" << reading->distanceMM
                 << "mm confidence=" << reading->confidence
                 << " angleErrorDeg=" << angleErrorDeg;
        result.debugLog.push_back(accepted.str());

        if (sensor.targetWall == Wall::LEFT || sensor.targetWall == Wall::RIGHT) {
            const auto correctedX = calculateXFromSensor(absolutePose, sensor, correctedDistanceInches);
            if (correctedX.has_value()) {
                candidates.x.push_back(*correctedX);
            }
        } else {
            const auto correctedY = calculateYFromSensor(absolutePose, sensor, correctedDistanceInches);
            if (correctedY.has_value()) {
                candidates.y.push_back(*correctedY);
            }
        }
    }

    return candidates;
}

}  // namespace wallsnap
