#include "wallsnap/wallsnap.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool near(double actual, double expected, double tolerance = 1e-6) {
    return std::abs(actual - expected) <= tolerance;
}

wallsnap::SensorConfig fakeSensor(
    pros::Distance& distance,
    wallsnap::Wall wall,
    double offsetX,
    double offsetY,
    double angleOffsetDeg) {
    wallsnap::SensorConfig sensor;
    sensor.sensor = &distance;
    sensor.offsetX = offsetX;
    sensor.offsetY = offsetY;
    sensor.angleOffsetDeg = angleOffsetDeg;
    sensor.targetWall = wall;
    sensor.name = "fake";
    return sensor;
}

wallsnap::WallSnapConfig baseConfig() {
    wallsnap::WallSnapConfig config;
    config.field.fieldWidth = 144.0;
    config.field.fieldHeight = 144.0;
    config.field.origin = wallsnap::FieldOrigin::BOTTOM_LEFT;
    config.coordinateMode = wallsnap::CoordinateMode::ABSOLUTE_FIELD;
    config.samples = 7;
    config.minDistanceMM = 50;
    config.maxDistanceMM = 2000;
    config.minConfidence = 35;
    config.maxAngleErrorDeg = 12.0;
    config.maxDistanceErrorInches = 12.0;
    config.blend = 1.0;
    return config;
}

void correctsXFromRightWall() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    pros::Distance right({965, 965, 965});
    snap.addSensor(fakeSensor(right, wallsnap::Wall::RIGHT, 5.0, 0.0, -90.0));

    const auto result = snap.calculateCorrection({101.0, 60.0, 0.0});
    assert(result.success);
    assert(result.correctedXValid);
    assert(!result.correctedYValid);
    assert(near(result.correctedX, 101.0, 0.02));
    assert(near(result.correctedAbsoluteX, 101.0, 0.02));
}

void correctsYFromFrontWall() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    pros::Distance front({991, 991, 991});
    snap.addSensor(fakeSensor(front, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));

    const auto result = snap.calculateCorrection({80.0, 100.0, 0.0});
    assert(result.success);
    assert(result.correctedYValid);
    assert(near(result.correctedY, 100.0, 0.02));
}

void averagesTwoXCorrections() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    pros::Distance left({1702, 1702, 1702});
    pros::Distance right({1702, 1702, 1702});
    snap.addSensor(fakeSensor(left, wallsnap::Wall::LEFT, -5.0, 0.0, 90.0));
    snap.addSensor(fakeSensor(right, wallsnap::Wall::RIGHT, 5.0, 0.0, -90.0));

    const auto result = snap.calculateCorrection({72.0, 60.0, 0.0});
    assert(result.success);
    assert(result.correctedXValid);
    assert(near(result.correctedX, 72.0, 0.02));
}

void supportsFieldCenterOrigin() {
    auto config = baseConfig();
    config.field.origin = wallsnap::FieldOrigin::FIELD_CENTER;
    wallsnap::WallSnap snap(config);
    pros::Distance left({432, 432, 432});
    snap.addSensor(fakeSensor(left, wallsnap::Wall::LEFT, -5.0, 0.0, 90.0));

    const auto result = snap.calculateCorrection({-50.0, 0.0, 0.0});
    assert(result.success);
    assert(result.correctedXValid);
    assert(near(result.correctedX, -50.0, 0.02));
}

void convertsStartRelativeCoordinates() {
    auto config = baseConfig();
    config.coordinateMode = wallsnap::CoordinateMode::START_RELATIVE;
    config.startAbsolutePose = {12.0, 12.0, 0.0};
    config.maxDistanceMM = 4000;
    wallsnap::WallSnap snap(config);
    pros::Distance front({2921, 2921, 2921});
    snap.addSensor(fakeSensor(front, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));

    const auto result = snap.calculateCorrection({0.0, 12.0, 0.0});
    assert(result.success);
    assert(result.correctedYValid);
    assert(near(result.correctedAbsoluteY, 24.0, 0.02));
    assert(near(result.correctedY, 12.0, 0.02));
}

void usesMedianReading() {
    auto config = baseConfig();
    config.samples = 5;
    wallsnap::WallSnap snap(config);
    pros::Distance front({100, 991, 991, 991, 1800});
    snap.addSensor(fakeSensor(front, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));

    const auto result = snap.calculateCorrection({80.0, 100.0, 0.0});
    assert(result.success);
    assert(result.correctedYValid);
    assert(near(result.correctedY, 100.0, 0.02));
}

void rejectsNullSensor() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    wallsnap::SensorConfig sensor;
    sensor.name = "missing";
    sensor.targetWall = wallsnap::Wall::FRONT;
    snap.addSensor(sensor);

    const auto result = snap.calculateCorrection({80.0, 100.0, 0.0});
    assert(!result.success);
}

void rejectsOutOfRangeAndLowConfidence() {
    auto config = baseConfig();
    pros::Distance nearDistance({40});
    wallsnap::WallSnap tooNear(config);
    tooNear.addSensor(fakeSensor(nearDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(!tooNear.calculateCorrection({80.0, 100.0, 0.0}).success);

    pros::Distance farDistance({2100});
    wallsnap::WallSnap tooFar(config);
    tooFar.addSensor(fakeSensor(farDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(!tooFar.calculateCorrection({80.0, 100.0, 0.0}).success);

    pros::Distance lowConfidenceDistance({991}, {10});
    wallsnap::WallSnap lowConfidence(config);
    lowConfidence.addSensor(fakeSensor(lowConfidenceDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(!lowConfidence.calculateCorrection({80.0, 100.0, 0.0}).success);
}

void rejectsWrongFacingSensor() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    pros::Distance front({991});
    snap.addSensor(fakeSensor(front, wallsnap::Wall::FRONT, 0.0, 5.0, 90.0));

    const auto result = snap.calculateCorrection({80.0, 100.0, 0.0});
    assert(!result.success);
}

void rejectsBlockedObjectDistance() {
    auto config = baseConfig();
    wallsnap::WallSnap snap(config);
    pros::Distance front({254, 254, 254});
    snap.addSensor(fakeSensor(front, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));

    const auto result = snap.calculateCorrection({80.0, 100.0, 0.0});
    assert(!result.success);
}

void appliesSoftCorrectionBlend() {
    auto config = baseConfig();
    pros::Distance noneDistance({864});
    wallsnap::WallSnap noBlend(config);
    noBlend.addSensor(fakeSensor(noneDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(near(noBlend.calculateSoftCorrection({80.0, 100.0, 0.0}, 0.0).correctedY, 100.0));

    pros::Distance halfDistance({864});
    wallsnap::WallSnap halfBlend(config);
    halfBlend.addSensor(fakeSensor(halfDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(near(halfBlend.calculateSoftCorrection({80.0, 100.0, 0.0}, 0.5).correctedY, 102.5, 0.02));

    pros::Distance fullDistance({864});
    wallsnap::WallSnap fullBlend(config);
    fullBlend.addSensor(fakeSensor(fullDistance, wallsnap::Wall::FRONT, 0.0, 5.0, 0.0));
    assert(near(fullBlend.calculateSoftCorrection({80.0, 100.0, 0.0}, 1.0).correctedY, 105.0, 0.02));
}

}  // namespace

int main() {
    correctsXFromRightWall();
    correctsYFromFrontWall();
    averagesTwoXCorrections();
    supportsFieldCenterOrigin();
    convertsStartRelativeCoordinates();
    usesMedianReading();
    rejectsNullSensor();
    rejectsOutOfRangeAndLowConfidence();
    rejectsWrongFacingSensor();
    rejectsBlockedObjectDistance();
    appliesSoftCorrectionBlend();
    return 0;
}
