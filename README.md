# WallSnap

WallSnap is a PROS C++17 VEX V5 library for correcting localization drift with
four VEX distance sensors and known field walls.

It does not implement motion control, PID, pure pursuit, path following, or full
odometry. WallSnap only calculates corrected coordinates so your code can pass
them into LemLib, EZ-Template, or your own odometry system.

The project is laid out like a PROS template library: public headers live under
`include/wallsnap`, implementation files live under `src/wallsnap`, and the
Makefile declares `IS_LIBRARY:=1` so it can be packaged in the same style as
LemLib when used with a PROS project.

## Install

Add the WallSnap depot once:

```sh
pros c add-depot WallSnap https://raw.githubusercontent.com/LeonCai100/WallSnap/depot/stable.json
```

Apply WallSnap to a PROS V5 project:

```sh
pros c apply WallSnap
```

Update WallSnap later:

```sh
pros c upgrade WallSnap
```

Then include the umbrella header:

```cpp
#include "wallsnap/wallsnap.hpp"
```

## Why Wall Resets Help

Tracking wheels, IMUs, and motor encoders drift over an autonomous run. Field
walls do not drift. If a distance sensor has a clear view of a wall, WallSnap
can compare the measured wall distance against the expected distance from your
current pose and return a corrected `x`, `y`, or both.

## Coordinate Modes

WallSnap always performs wall math internally in absolute field coordinates
because walls are fixed field objects.

In `ABSOLUTE_FIELD` mode, your pose is already measured in field coordinates.
You can use either a bottom-left origin or a field-center origin.

In `START_RELATIVE` mode, your pose is measured from the robot's autonomous
starting position. You provide `startAbsolutePose`, and WallSnap converts your
pose to absolute coordinates, calculates corrections, then converts the result
back to start-relative coordinates.

## Units And Conventions

- Position units are inches.
- VEX distance sensor readings are millimeters and are converted internally.
- `theta` is heading in degrees.
- `offsetX` is positive to the robot's right.
- `offsetY` is positive to the robot's front.
- `angleOffsetDeg = 0` is front-facing.
- `angleOffsetDeg = 90` is left-facing.
- `angleOffsetDeg = -90` is right-facing.
- `angleOffsetDeg = 180` is back-facing.

For a bottom-left origin, left/right/back/front walls are `x = 0`,
`x = fieldWidth`, `y = 0`, and `y = fieldHeight`. For a field-center origin,
they are `x = -fieldWidth / 2`, `x = fieldWidth / 2`,
`y = -fieldHeight / 2`, and `y = fieldHeight / 2`.

## Sensor Mounting

Measure each sensor from the robot tracking center, not from the edge of the
robot. Use the center of the distance sensor's face as the measurement point.

Four common mounts are:

```cpp
wallsnap::SensorConfig front;
front.sensor = &frontDist;
front.offsetX = 0.0;
front.offsetY = 5.0;
front.angleOffsetDeg = 0.0;
front.targetWall = wallsnap::Wall::FRONT;
front.name = "front";
snap.addSensor(front);
```

## Complete Example

```cpp
#include "wallsnap/wallsnap.hpp"

#include "pros/distance.hpp"

pros::Distance frontDist(1);
pros::Distance backDist(2);
pros::Distance leftDist(3);
pros::Distance rightDist(4);

wallsnap::WallSnapConfig config;
config.field.fieldWidth = 144.0;
config.field.fieldHeight = 144.0;
config.field.origin = wallsnap::FieldOrigin::BOTTOM_LEFT;
config.coordinateMode = wallsnap::CoordinateMode::ABSOLUTE_FIELD;
config.samples = 7;
config.minConfidence = 35;
config.maxAngleErrorDeg = 12.0;
config.blend = 1.0;

wallsnap::WallSnap snap(config);

wallsnap::SensorConfig front;
front.sensor = &frontDist;
front.offsetX = 0.0;
front.offsetY = 5.0;
front.angleOffsetDeg = 0.0;
front.targetWall = wallsnap::Wall::FRONT;
front.name = "front";
snap.addSensor(front);

wallsnap::SensorConfig back;
back.sensor = &backDist;
back.offsetX = 0.0;
back.offsetY = -5.0;
back.angleOffsetDeg = 180.0;
back.targetWall = wallsnap::Wall::BACK;
back.name = "back";
snap.addSensor(back);

wallsnap::SensorConfig left;
left.sensor = &leftDist;
left.offsetX = -5.0;
left.offsetY = 0.0;
left.angleOffsetDeg = 90.0;
left.targetWall = wallsnap::Wall::LEFT;
left.name = "left";
snap.addSensor(left);

wallsnap::SensorConfig right;
right.sensor = &rightDist;
right.offsetX = 5.0;
right.offsetY = 0.0;
right.angleOffsetDeg = -90.0;
right.targetWall = wallsnap::Wall::RIGHT;
right.name = "right";
snap.addSensor(right);

wallsnap::Pose pose = {currentX, currentY, currentThetaDeg};
wallsnap::ResetResult result = snap.calculateCorrection(pose);

if (result.success) {
    if (result.correctedXValid) currentX = result.correctedX;
    if (result.correctedYValid) currentY = result.correctedY;
}
```

## Start-Relative Example

```cpp
wallsnap::WallSnapConfig config;
config.field.fieldWidth = 144.0;
config.field.fieldHeight = 144.0;
config.field.origin = wallsnap::FieldOrigin::BOTTOM_LEFT;
config.coordinateMode = wallsnap::CoordinateMode::START_RELATIVE;
config.startAbsolutePose = {12.0, 12.0, 0.0};
```

With this setup, an input pose of `{0, 0, 0}` means the robot is at its starting
position, but WallSnap still uses the real field walls internally.

## LemLib Usage

Keep LemLib in charge of odometry. Use WallSnap only to calculate replacement
coordinates during moments where a reset is safe.

```cpp
auto pose = chassis.getPose();
wallsnap::ResetResult result = snap.calculateCorrection({pose.x, pose.y, pose.theta});

if (result.success) {
    double x = result.correctedXValid ? result.correctedX : pose.x;
    double y = result.correctedYValid ? result.correctedY : pose.y;
    chassis.setPose(x, y, pose.theta);
}
```

## Why Resets Fail

Check `ResetResult::debugLog` when `success` is false or only one axis is valid.
Common causes are:

- the distance is outside `minDistanceMM` or `maxDistanceMM`
- the sensor confidence is below `minConfidence`
- the robot is not facing the configured wall closely enough
- the sensor sees a game object, robot, or field element instead of the wall
- the configured sensor offset or target wall is wrong

Use `calculateSoftCorrection(pose, 0.3)` or set `config.blend` below `1.0` if
you want gentle corrections instead of hard coordinate resets.

## Publishing A Release

WallSnap follows LemLib's release pattern:

1. Update `VERSION` in `Makefile`.
2. Commit the change.
3. Tag the release, for example `v0.1.0`.
4. Push `main` and the tag.

GitHub Actions builds `WallSnap@VERSION.zip`, uploads it to the GitHub Release,
and publishes `stable.json` to the `depot` branch. Users install from that depot
with the commands above.
