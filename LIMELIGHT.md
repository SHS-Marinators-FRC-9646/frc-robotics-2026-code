# Limelight + AprilTag Auto-Align (Proof of Concept)

Hold the **right bumper** on the Xbox controller and the robot drives itself to a spot
0.75 m in front of the nearest AprilTag, facing it. Let go and the driver is back in control.

This doc explains how that works, how to set up the cameras, and how to test it.

---

## 1. The big picture

The robot answers two questions every 20 ms:

1. **Where am I on the field?** (localization)
2. **How do I get to where I want to be?** (control)

```
  Swerve wheels + gyro ──► odometry (250 Hz, drifts over time)
                                │
                                ▼
                      ┌──────────────────┐
  Limelight(s) ──────►│  Pose estimator  │──► best guess of robot pose (x, y, heading)
  (AprilTags)         │  (Kalman filter) │            │
                      └──────────────────┘            ▼
                                             DriveToPose command
                                                      │
                                                      ▼
                                          swerve velocity requests
```

- **Odometry** integrates wheel motion and the gyro. It is smooth and fast but slowly
  drifts (wheels slip, robots get bumped).
- **Vision** looks at AprilTags whose positions on the field are known exactly, and
  computes where the robot must be to see them that way. It doesn't drift, but it is
  noisy and arrives late (camera latency).
- The **pose estimator** blends the two. Each vision measurement comes with a "standard
  deviation" saying how much to trust it. CTRE's swerve library already includes this
  estimator; we just call `drivetrain.addVisionMeasurement(...)`.

If you know cloud systems: think of odometry as a fast local cache that goes stale, and
vision as a slower, authoritative source that periodically corrects it.

### Coordinate system

All poses use WPILib's **blue-origin** field coordinates, regardless of our alliance:

- Origin is the corner of the field on the blue alliance wall, on the right side as seen from the blue driver station.
- **+X** points away from the blue wall toward the red wall.
- **+Y** points to the left (as seen from the blue driver station).
- Angles are counter-clockwise positive; 0° faces +X.

Using one fixed frame everywhere avoids a whole class of red/blue bugs.

### MegaTag1 vs MegaTag2

Limelight offers two ways to compute the robot's pose from tags:

| | MegaTag1 | MegaTag2 |
|---|---|---|
| Uses | Tags only | Tags + our gyro heading |
| Gives heading? | Yes | No (uses ours) |
| Single-tag accuracy | Poor (flips/jumps) | Good |
| Needs correct gyro? | No | **Yes** |

We use **MegaTag2 while enabled** (it's much more stable) and **MegaTag1 while
disabled, only when 2+ tags are visible**, to fix the gyro's heading before a match
starts. That matters because MegaTag2 assumes our heading is already right.

---

## 2. Code tour

| File | What it does |
|---|---|
| `src/main/java/frc/robot/LimelightHelpers.java` | Limelight's official helper library (v1.14, needs LLOS 2026.0+). Reads camera data from NetworkTables. Don't edit; replace with a newer release when needed. |
| `src/main/java/frc/robot/subsystems/Vision.java` | Every loop: sends our heading to each Limelight, reads its pose estimate, filters out bad ones, and passes good ones to the drivetrain's pose estimator. |
| `src/main/java/frc/robot/commands/DriveToPose.java` | Drives to a target pose with three motion-profiled PID controllers (x, y, heading). Ends when within 2 cm and 1°. |
| `src/main/java/frc/robot/RobotContainer.java` | Creates `Vision` with the camera names, computes "in front of the nearest tag", and binds it to the right bumper. |
| `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` | Existing CTRE swerve code. Its `addVisionMeasurement` override converts Limelight timestamps to CTRE's clock; nothing to change. |

### Filtering rules in `Vision`

A vision measurement is thrown away when:

- No tags are visible (`tagCount == 0`).
- The robot is spinning faster than 720°/s (the camera frame and gyro reading disagree too much).
- Disabled and fewer than 2 tags are visible (MegaTag1 is unreliable with one tag).

### How `DriveToPose` moves the robot

For each axis, a **trapezoid profile** plans a smooth speed curve (accelerate, cruise,
decelerate) from the current position to the goal, and a **P controller** corrects
whatever error remains. Output speed is `profile velocity + P × error`. Limits are
deliberately low for testing (1 m/s, 180°/s); they're constants at the top of the file.

It uses `ForwardPerspectiveValue.BlueAlliance` so "+X" means the field's +X, not
"away from whichever driver station we're in."

---

## 3. Hardware and camera setup (do once per camera)

Our team number is **9646**, so robot network addresses are `10.96.46.x`.

1. **Mount** the camera rigidly. Any wobble becomes pose error. Tilt it so tags are in
   view from where you'll typically be.
2. **Power and network.** Follow Limelight's wiring guide for your model (12 V from the
   PDH or PoE). Connect Ethernet to the robot radio. With two cameras plus the roboRIO,
   you'll usually need a small Ethernet switch.
3. **Update the firmware (LLOS)** to 2026.0 or later using the Limelight Hardware
   Manager. Our `LimelightHelpers` requires it.
4. **Open the web UI** at `http://limelight.local:5801` (or the camera's IP, port 5801).
5. **Settings tab:**
   - **Hostname:** e.g. `limelight-front`, `limelight-back`. This must match the names in
     `RobotContainer.java` exactly; it's how the code finds each camera's data.
   - **Static IP:** `10.96.46.11`, `10.96.46.12`, ... (one per camera). Static IPs make
     connections faster and more reliable at competitions.
6. **Pipeline tab:**
   - Pipeline type: **AprilTag** (fiducial markers), family **36h11**.
   - Field map: the **2026** map (built in on current LLOS; otherwise upload the
     `.fmap` file from Limelight's site).
   - Enable **Full 3D** targeting.
7. **Camera pose in robot space** (3D / Robot tab). Measure from the **robot's center on
   the floor** to the camera lens:
   - Forward (m), Side (m, left is +), Up (m)
   - Roll, Pitch, Yaw (degrees; pitch up is +)

   **This is the #1 source of bad poses.** Measure carefully, and double-check the signs.

Then update the names in `RobotContainer.java`:

```java
public final Vision vision = new Vision(drivetrain, "limelight-front", "limelight-back");
```

One camera works fine too. Just pass one name.

---

## 4. Test plan

Work through these in order. Each step proves one layer before adding the next.

### Step 0: Simulation (no robot needed)

```
./gradlew simulateJava
```

In the sim GUI, map a keyboard or plugged-in controller to Joystick 0, then enable
Teleop. The simulated robot starts at the field origin, and holding right bumper should
drive it in front of the nearest tag. There are no cameras in sim, so this tests
`DriveToPose` and the target math only.

Watch it in **AdvantageScope** (bundled with WPILib): connect to the sim, open a 2D
Field view, and add `DriveState/Pose` (robot) and `DriveToPose/Target` (goal).

### Step 1: Cameras see tags (robot on blocks or disabled)

- In each camera's web UI, confirm tags are detected with stable IDs.
- In AdvantageScope, add `Vision/<camera-name>/Pose` to the field view. Hold a tag in
  front of the camera. The pose should be **stable** (not jumping) and roughly where the
  robot actually is.
- With two cameras, both poses should agree within a few centimeters.

### Step 2: Heading seeding (disabled)

Place the robot where a camera sees 2+ tags. While disabled, `DriveState/Pose` should
snap to the correct location and heading within a second or two.

### Step 3: Fusion while driving (enabled, slow)

Drive around by hand. `DriveState/Pose` should track smoothly, and when you look at tags,
small corrections should pull it back in line. No big jumps.

### Step 4: Auto-align (enabled, clear space, hand on disable)

Start about 2 m from a tag and hold right bumper. The robot should drive slowly to 0.75 m
in front of the tag, square to it, and stop. Release the bumper at any time to abort.

---

## 5. Tuning, in this order

1. **Run SysId on the drivetrain** (already bound: Back/Start + X/Y). Accurate drive
   gains make everything downstream better.
2. **Vision trust** (`Vision.java`): if the pose jitters, raise `kMegaTag2XYStdDev`.
   If it's slow to correct drift, lower it.
3. **DriveToPose gains** (`DriveToPose.java`): if it overshoots or oscillates near the
   goal, lower `kTranslationP` / `kHeadingP`. If it stalls just short, raise them.
   Raise the speed limits only once it's well-behaved.
4. **Alignment distance** (`kAlignDistanceFromTagMeters` in `RobotContainer.java`):
   it's measured from the tag to the robot's **center**, so it must be at least half the
   bumper-to-bumper length.

---

## 6. Known limitations of this POC

- **"Nearest tag" is naive.** It picks the closest tag on the whole field, including ones
  you may not want (wrong alliance, wrong side of a field element). For real use, pick
  specific tag IDs per task/alliance, e.g. `fieldLayout.getTagPose(id)`.
- **No obstacle avoidance.** `DriveToPose` drives in a straight line. For long distances
  across the field, consider PathPlanner's `AutoBuilder.pathfindToPose(...)` to get close,
  then `DriveToPose` for the final alignment. This needs the PathPlanner vendor library,
  and the values in `src/main/deploy/pathplanner/settings.json` would need to be fixed to
  match `TunerConstants.java` first.
- **Driver controls are unchanged:** still robot-centric, and capped at 10% speed in
  `RobotContainer.java` (`MaxSpeed`).
- **Gains are placeholders** until tested on the real robot.
- **Limelight 4 IMU:** the LL4 has a built-in IMU that can improve MegaTag2 further
  (`LimelightHelpers.SetIMUMode`). Not used yet.

## Further reading

- [Limelight docs: MegaTag2 and robot localization](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
- [WPILib: pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [WPILib: coordinate system](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
- [CTRE swerve: vision measurements](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-pose.html)
- [AdvantageScope](https://docs.advantagescope.org/)
