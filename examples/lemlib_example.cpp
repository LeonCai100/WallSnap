#include "wallsnap/wallsnap.hpp"

#include "pros/distance.hpp"

#include <string>

// This file intentionally does not include LemLib headers. The important part
// is that WallSnap returns coordinates; your robot code decides how to apply
// them to LemLib's chassis or pose object.

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

wallsnap::WallSnap makeLemLibWallSnap() {
    wallsnap::WallSnapConfig config;
    config.field.fieldWidth = 144.0;
    config.field.fieldHeight = 144.0;
    config.field.origin = wallsnap::FieldOrigin::BOTTOM_LEFT;
    config.coordinateMode = wallsnap::CoordinateMode::ABSOLUTE_FIELD;
    config.blend = 0.75;

    wallsnap::WallSnap snap(config);
    snap.addSensor(makeSensor(frontDist, 0.0, 5.0, 0.0, wallsnap::Wall::FRONT, "front"));
    snap.addSensor(makeSensor(backDist, 0.0, -5.0, 180.0, wallsnap::Wall::BACK, "back"));
    snap.addSensor(makeSensor(leftDist, -5.0, 0.0, 90.0, wallsnap::Wall::LEFT, "left"));
    snap.addSensor(makeSensor(rightDist, 5.0, 0.0, -90.0, wallsnap::Wall::RIGHT, "right"));
    return snap;
}

void applyLemLibStyleExample(double currentX, double currentY, double currentThetaDeg) {
    wallsnap::WallSnap snap = makeLemLibWallSnap();
    wallsnap::ResetResult result = snap.calculateCorrection({currentX, currentY, currentThetaDeg});

    if (!result.success) {
        return;
    }

    const double newX = result.correctedXValid ? result.correctedX : currentX;
    const double newY = result.correctedYValid ? result.correctedY : currentY;

    // Example:
    // chassis.setPose(newX, newY, currentThetaDeg);
    (void)newX;
    (void)newY;
}
