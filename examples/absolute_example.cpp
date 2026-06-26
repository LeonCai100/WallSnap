#include "wallsnap/wallsnap.hpp"

#include "pros/distance.hpp"

#include <string>

pros::Distance frontDist(1);
pros::Distance backDist(2);
pros::Distance leftDist(3);
pros::Distance rightDist(4);

wallsnap::SensorConfig makeSensor(
    pros::Distance& distance,
    double offsetX,
    double offsetY,
    double angleOffsetDeg,
    wallsnap::Wall targetWall,
    const std::string& name) {
    wallsnap::SensorConfig sensor;
    sensor.sensor = &distance;
    sensor.offsetX = offsetX;
    sensor.offsetY = offsetY;
    sensor.angleOffsetDeg = angleOffsetDeg;
    sensor.targetWall = targetWall;
    sensor.name = name;
    return sensor;
}

wallsnap::WallSnap makeAbsoluteWallSnap() {
    wallsnap::WallSnapConfig config;
    config.field.fieldWidth = 144.0;
    config.field.fieldHeight = 144.0;
    config.field.origin = wallsnap::FieldOrigin::BOTTOM_LEFT;
    config.coordinateMode = wallsnap::CoordinateMode::ABSOLUTE_FIELD;
    config.samples = 7;
    config.minConfidence = 35;
    config.maxAngleErrorDeg = 12.0;

    wallsnap::WallSnap snap(config);
    snap.addSensor(makeSensor(frontDist, 0.0, 5.0, 0.0, wallsnap::Wall::FRONT, "front"));
    snap.addSensor(makeSensor(backDist, 0.0, -5.0, 180.0, wallsnap::Wall::BACK, "back"));
    snap.addSensor(makeSensor(leftDist, -5.0, 0.0, 90.0, wallsnap::Wall::LEFT, "left"));
    snap.addSensor(makeSensor(rightDist, 5.0, 0.0, -90.0, wallsnap::Wall::RIGHT, "right"));
    return snap;
}

void applyAbsoluteExample(double& currentX, double& currentY, double currentTheta) {
    wallsnap::WallSnap snap = makeAbsoluteWallSnap();
    wallsnap::ResetResult result = snap.calculateCorrection({currentX, currentY, currentTheta});

    if (result.success) {
        if (result.correctedXValid) {
            currentX = result.correctedX;
        }
        if (result.correctedYValid) {
            currentY = result.correctedY;
        }
    }
}
