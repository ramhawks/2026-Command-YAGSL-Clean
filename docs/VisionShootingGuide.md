# Vision-Assisted Shooting — Student Guide
### 2026 Robot | Limelight + AprilTag

---

## What Does This Code Do?

When the driver holds the **Y button**, the robot automatically runs a three-step
sequence to find, aim at, and shoot into a target that has an AprilTag on it:

```
Step 1: DRIVE    -->  Step 2: AIM    -->  Step 3: SHOOT
Drive toward            Rotate until          Spin up &
target until            centered on           launch
tag is seen             the tag
```

If the driver releases the Y button at any point, the sequence immediately stops
and the robot returns to manual control.

---

## Background: What is a Limelight?

A **Limelight** is a small smart camera designed for FRC robots. It connects over
the robot's network and publishes data to a system called NetworkTables that our
code can read. The key values we use are:

| Value | What It Means |
|-------|--------------|
| `tv`  | **Target Valid** — `1` if a target is visible, `0` if not |
| `tx`  | **Target X** — how many degrees left/right the target is from the camera's crosshair. Negative = target is LEFT, Positive = target is RIGHT |
| `ty`  | **Target Y** — how many degrees up/down the target is (we don't use this here) |

We access these values using a helper library called `LimelightHelpers`:
```java
LimelightHelpers.getTV("limelight")   // returns true/false
LimelightHelpers.getTX("limelight")   // returns degrees, e.g. -5.2
```

---

## What is an AprilTag?

An **AprilTag** is a printed black-and-white square (like a QR code) that the
Limelight can detect and identify. Each tag has a unique ID number. In FRC, they
are placed around the field on scoring targets. Our code doesn't care which ID
the tag has — it just needs to see ANY valid tag to trigger the sequence.

---

## Step 1 — DriveToTargetCommand.java

**Goal:** Drive the robot straight forward until the Limelight sees the AprilTag.

### How it works:

```
initialize()  -->  Print a message so we know the command started
execute()     -->  Check if the Limelight sees a target (getTV)
                   - If NO target: drive forward at 0.75 m/s
                   - If YES target: stop immediately
isFinished()  -->  Return true as soon as getTV() is true
end()         -->  Stop the robot; print whether we found a target or timed out
```

### The key logic in execute():
```java
boolean hasTarget = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);

if (!hasTarget) {
    swerve.setChassisSpeeds(new ChassisSpeeds(DRIVE_SPEED_MPS, 0.0, 0.0));
} else {
    swerve.setChassisSpeeds(new ChassisSpeeds());
}
```

`ChassisSpeeds(vx, vy, omega)` tells the swerve drive how to move:
- `vx` = forward/backward speed in meters per second
- `vy` = left/right strafe speed (0 = no strafe)
- `omega` = rotation speed in radians per second (0 = no rotation)

A 10-second timeout is applied externally so the robot does not drive forever
if the tag is never found.

### ChassisSpeeds(vx, vy, omega) VS. swerve.drive(new Translation2d(vx, vy), omega, false)

What is the difference between ChassisSpeeds(vx, vy, omega) and drive(Translation2d translation, double rotation, boolean fieldRelative)?

Both move the robot, but speak different "languages" to the drivetrain:

**ChassisSpeeds(vx, vy, omega)**

This method takes a velocity directly — you hand it exact m/s values for X, Y, and rotation. It's a low-level, direct command with no processing in between. You tell it exactly what speed you want and it does it.

**swerve.drive(Translation2d translation, double rotation, boolean fieldRelative)**

This method is a higher-level YAGSL method. Under the hood it does extra work before sending speeds to the modules — it applies your configured speed limits, handles field-relative rotation math using the gyro heading, and feeds through YAGSL's internal velocity processing pipeline. This is what your default teleop drive command uses internally.

### What to watch on SmartDashboard:
- `DriveToTarget/HasTarget` — turns `true` the moment the tag is spotted

---

## Step 2 — AimAtHubCommand.java

**Goal:** Rotate the robot in place until the target is centered in the camera's
view (tx is close to 0 degrees).

### How it works:

The command uses a **proportional controller (P controller)**. This is the
simplest form of feedback control:

```
Error = how far off we are (tx degrees)
Output = kP * Error   (a small constant times the error)
```

When the error is large, the output is large (rotate fast).
When the error is small, the output is small (rotate slow).
When the error reaches zero, the output reaches zero (stop).

```java
double tx = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
double rawOutput = -kP * tx;
```

The negative sign is important: if `tx` is positive (target is to the RIGHT),
we need to rotate RIGHT, which in WPILib's convention is a negative omega value.

### Minimum output:
Static friction can prevent the robot from moving when the error is very small.
To fix this, we enforce a minimum output:
```java
if (Math.abs(rawOutput) < kMinOutput) {
    omega = Math.copySign(kMinOutput, rawOutput);
}
```
`Math.copySign(a, b)` returns the value `a` with the same sign as `b`. So we
keep the direction of movement but bump up the power to the minimum.

### Finishing condition:
The command doesn't end the first time it hits the tolerance — it requires
**5 consecutive loops** (every loop = 20 ms, so 100 ms total) within tolerance
before declaring success. This prevents ending on a single lucky frame.

### What to watch on SmartDashboard:
- `AimAtHub/TX` — should count down toward 0 as the robot rotates
- `AimAtHub/HasTarget` — should stay `true` throughout this step
- `AimAtHub/StableLoops` — counts up to 5 as the robot holds alignment

---

## Step 3 — Shoot Sequence

**Goal:** Spin the launcher up to speed, then fire.

```java
ballSubsystem.spinUpCommand().until(ballSubsystem::launcherAtSpeed).withTimeout(1.5),
ballSubsystem.launchCommand().withTimeout(3.0)
```

Two sub-steps:
1. **Spin Up** — runs the launcher motor and waits until the RPM sensor reads
   at least 4500 RPM (within ±150 RPM, held for 0.1 seconds). If it hasn't
   reached speed within 1.5 seconds, it moves on anyway.
2. **Launch** — runs both the feeder and launcher motors for up to 3 seconds.

---

## How It All Connects — RobotContainer.java

In `RobotContainer.java`, the three steps are chained together using
`Commands.sequence()`. This runs them one at a time, each starting only after
the previous one finishes:

```java
private Command buildDriveAimShootCommand() {
    Command driveToTarget = new DriveToTargetCommand(m_swerveSubsystem).withTimeout(10.0);
    Command aimAtTarget   = new AimAtHubCommand(m_swerveSubsystem);
    Command shoot         = Commands.sequence(
        ballSubsystem.spinUpCommand().until(ballSubsystem::launcherAtSpeed).withTimeout(1.5),
        ballSubsystem.launchCommand().withTimeout(3.0)
    );
    return Commands.sequence(driveToTarget, aimAtTarget, shoot);
}
```

This method is called when the driver holds the **Y button**:
```java
m_driverController.y().whileTrue(buildDriveAimShootCommand());
```

`whileTrue` means: start the sequence when Y is pressed, cancel it the moment
Y is released.

---

## Tuning Guide

These are the values you should adjust during testing. Change ONE value at a
time, test, and observe the result before changing another.

---

### DriveToTargetCommand.java

| Constant | Location | Current Value | What To Do |
|----------|----------|---------------|------------|
| `DRIVE_SPEED_MPS` | Line ~40 | `0.75` | If the robot drives past the tag before stopping, **lower** this value. If it is too slow, raise it. Stay under 1.5 m/s. |

**How to test Step 1 in isolation:**
Place the robot 30 ft from the target. Hold Y. Watch `DriveToTarget/HasTarget`
on SmartDashboard. The robot should stop as soon as it flips to `true`. If it
stops too late (too close), lower `DRIVE_SPEED_MPS`.

---

### AimAtHubCommand.java

| Constant | Location | Current Value | What To Do |
|----------|----------|---------------|------------|
| `kP` | Line ~45 | `0.04` | If the robot barely moves when misaligned, **increase** by `0.01`. If it oscillates back and forth, **decrease** by `0.01`. |
| `kMinOutput` | Line ~50 | `0.02` | If the robot stalls when almost aligned (tx is small but robot won't finish rotating), **increase** slightly. |
| `kMaxOutput` | Line ~55 | `0.4` | Maximum rotation speed cap. Lower this if the robot overshoots badly. |
| `TOLERANCE_DEGREES` | Line ~58 | `1.5` | How precisely the robot must be centered. Tighten to `1.0` if shots are off. Loosen to `2.5` if it never finishes aligning. |
| `STABLE_LOOPS_REQUIRED` | Line ~61 | `5` | How many consecutive 20 ms loops must be within tolerance. Raise to `10` for more stability confirmation. Lower to `3` if it's too slow to confirm. |

**How to test Step 2 in isolation:**
Stand the robot in front of the target but offset to one side. Watch
`AimAtHub/TX` on SmartDashboard — it should smoothly approach 0 and the robot
should visibly rotate to face the target.

**Signs that kP is wrong:**
- Robot barely rotates → kP too low
- Robot spins back and forth past the target → kP too high
- Robot rotates in the wrong direction → you may have a camera mounting issue
  (check that the camera faces forward and is not inverted)

---

### Shoot Sequence (RobotContainer.java — buildDriveAimShootCommand)

| Parameter | Current Value | What To Do |
|-----------|---------------|------------|
| SpinUp timeout | `1.5 s` | Raise if the motor needs more time to reach speed before shooting |
| Launch timeout | `3.0 s` | Raise if more feed time is needed to empty the hopper |
| Target RPM | `4500 RPM` (in Constants.java `FuelConstants`) | Adjust based on how far you are from the target |

---

## Troubleshooting

**The robot drives past the target and doesn't stop.**
- Lower `DRIVE_SPEED_MPS` in `DriveToTargetCommand`
- Make sure the Limelight pipeline is set to AprilTag detection mode (not
  retroreflective). Check the Limelight web interface at `http://limelight.local:5801`

**The robot stops but doesn't rotate toward the target.**
- Open SmartDashboard and check `AimAtHub/HasTarget`. If it is `false`, the
  Limelight lost the tag when the robot stopped — the tag may be outside the
  camera's field of view at that distance. Drive closer or adjust the camera angle.

**The robot rotates but never finishes aligning (never shoots).**
- Lower `TOLERANCE_DEGREES` from `1.5` to `2.0` or `2.5`
- If `AimAtHub/TX` oscillates around 0 but never settles, lower `kP`

**The robot rotates the wrong direction.**
- Check `AimAtHub/TX` on SmartDashboard. If the sign is wrong relative to where
  the target visually is, the camera may be mounted backwards or rotated.
  Check Constants.java `LL_YAW_DEG` and `LL_ROLL_DEG`.

**The launcher doesn't fire.**
- Check `Fuel/LauncherRPM` on SmartDashboard. If it never reaches 4500, the
  `spinUpCommand` times out and launches anyway. Check motor connections.
- The operator and driver controllers share subsystem requirements — make sure
  the operator is not accidentally holding a button that conflicts.

---

## Key Files Reference

| File | Purpose |
|------|---------|
| `commands/DriveToTargetCommand.java` | Step 1: Drive until tag detected |
| `commands/AimAtHubCommand.java` | Step 2: Rotate until centered on tag |
| `subsystems/CANFuelSubsystem.java` | Step 3: Spin up and launch |
| `RobotContainer.java` | Wires the sequence to the Y button |
| `LimelightHelpers.java` | Library for reading Limelight camera data |
| `Constants.java` | Camera name, mounting position, RPM targets |

---

*2026 Robot Code — Chaotic Robotics*
