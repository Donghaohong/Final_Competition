# Code Overview

This document explains the current final competition codebase: what each file
does, how the modules connect, and which entry points should be used for testing.

The code is organized into these main layers:

1. Test entries and the future competition entry.
2. Initial localization.
3. EKF waypoint following.
4. PF waypoint following.
5. Known-map path planning.
6. Optional-wall verification.
7. Plotting and debugging.
8. Robot / simulator wrappers.
9. Math, geometry, and sensor-model helpers.

## Main Test Entries

### `testInitializeLocalizationPracticeMap.m`

PracticeMap entry for testing only initial localization.

Flow:

1. Load `PracticeMap2026.mat`.
2. Set the simulator camera offset.
3. Call `initializeLocalization`.
4. Save `latestInitLocalizationPracticeRun.mat`.

Use this when you only want to verify that the robot can identify its starting
waypoint and initial heading.

### `testEkfWaypointFollowingPracticeMap.m`

PracticeMap entry for EKF localization plus waypoint following.

Flow:

1. Load `PracticeMap2026.mat`.
2. Run `initializeLocalization` to get `initState`.
3. Use `planWaypointSequenceKnownMap` to build a collision-free path through the
   high-level goals.
4. Run `runEkfWaypointFollower`.
5. Save `latestEkfWaypointPracticeRun.mat`.

This is the current stable waypoint-following test entry.

### `testPfWaypointFollowingPracticeMap.m`

PracticeMap entry for PF localization plus waypoint following.

Flow:

1. Load `PracticeMap2026.mat`.
2. Run `initializeLocalization`.
3. Use the known-map planner to build a path.
4. Run `runPfWaypointFollower`.
5. Save `latestPfWaypointPracticeRun.mat`.

This is a parallel test path for A/B comparison against the EKF follower. It
does not replace the current integrated EKF flow.

### `testPfWaypointOptionalPracticeMap.m`

PracticeMap entry for testing PF waypoint following together with optional-wall
verification.

Flow:

1. Load `PracticeMap2026.mat`.
2. Run `initializeLocalization`.
3. Plan and execute waypoint following with `runPfWaypointFollower`.
4. Continue from the final PF state into `verifyOptionalWalls`.
5. Set `verifyParams.followerMode = 'pf'`, so optional-wall travel to
   observation/probe points uses the PF follower instead of the EKF follower.
6. Save `latestPfWaypointOptionalPracticeRun.mat`.

Use this to test the PF pipeline with optional-wall detection without modifying
the existing EKF integrated entry.

### `testPfWaypointEcOptionalPracticeMap.m`

PracticeMap entry for the PF three-stage strategy: normal waypoints first,
optional-wall verification second, EC waypoints last.

Flow:

1. Load `PracticeMap2026.mat`, including `ECwaypoints`.
2. Run `initializeLocalization`.
3. Visit normal waypoints with `runPfWaypointFollower`.
4. Run `verifyOptionalWalls` with `verifyParams.followerMode = 'pf'`.
5. Update `verifiedMap` with optional walls that were confirmed present.
6. Plan EC waypoints in the `ECwaypoints` file order from the post-verification
   PF pose, planned against `verifiedMap`.
7. Visit reachable EC waypoints with `runPfWaypointFollower` using
   `verifiedMap`.
8. Save `latestPfWaypointEcOptionalPracticeRun.mat`.

Use this to test the scoring-oriented sequence where normal waypoints are
completed first, optional walls are classified before EC planning, and EC paths
respect the updated map.

### `testOptionalWallVerificationPracticeMap.m`

PracticeMap entry for testing optional-wall verification by itself.

Flow:

1. Load the practice map.
2. Run initial localization.
3. Call `verifyOptionalWalls` to actively classify optional walls.
4. Save `latestOptionalWallPracticeRun.mat`.

Use this to debug the depth-plus-bump-probe optional-wall classifier.

### `testIntegratedWaypointOptionalPracticeMap.m`

Current integrated PracticeMap test entry.

Flow:

1. Load `PracticeMap2026.mat`.
2. Run initial localization.
3. Visit the remaining normal waypoints from the detected start.
4. Before each planned segment, check whether the path is relevant to an unknown
   optional wall.
5. If relevant, call `verifyOptionalWalls` first.
6. If the wall exists, add it to `verifiedMap` and replan.
7. After all waypoints are completed, verify remaining unknown optional walls.
8. Save `latestIntegratedPracticeRun.mat`.

This entry currently uses the EKF follower, not the PF follower.

## Future Competition Entry

### `finalCompetition.m`

Not implemented yet.

The required final competition entry is:

```matlab
function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
```

The competition entry must load `compMap.mat`, not `PracticeMap2026.mat`.

It should integrate the current modules:

- `initializeLocalization`
- `planWaypointSequenceKnownMap` / `planPathKnownMap`
- `runEkfWaypointFollower`
- `verifyOptionalWalls`
- final plotting
- global `dataStore`

## Initial Localization

### `initializeLocalization.m`

Main PF-based initial localization function.

Goals:

- Identify which `waypoints` entry the robot started from.
- Estimate the initial heading.
- Return an `initState` that can be handed to EKF or PF tracking.

Core logic:

1. Generate particles only near candidate `waypoints`.
2. Cover heading over `[-pi, pi)`.
3. Run a slow in-place sweep.
4. Propagate particles with odometry.
5. Score particles with beacons and known-map depth.
6. Estimate waypoint posterior and pose.
7. Check convergence.

Important constraint: initialization does not use `optWalls`.

### `initDefaultParams.m`

Default parameters for initial localization.

Includes:

- number of particles;
- heading bins;
- sweep timing and angular velocity;
- PF motion noise;
- depth likelihood parameters;
- beacon likelihood parameters;
- resampling parameters;
- convergence thresholds.

### `initializeParticlesFromWaypoints.m`

Creates the initial particle set from candidate start waypoints.

Key behavior:

- Particles are concentrated near candidate starts.
- Each waypoint receives a portion of the particles.
- Heading is spread around the full circle.

### `runInitializationSweep.m`

Runs the initial sweep loop.

Each loop:

1. Sends the sweep command.
2. Reads odometry.
3. Reads depth and beacon observations.
4. Propagates particles.
5. Computes likelihoods.
6. Updates weights.
7. Estimates pose.
8. Checks convergence.
9. Resamples when needed.
10. Logs to `dataStore.init`.

### `commandInitializationSweep.m`

Motion-command wrapper for initialization.

It sends a conservative in-place rotation command:

```matlab
SetFwdVelAngVelCreate(Robot, v, w)
```

### `getInitializationObservations.m`

Observation wrapper for initialization.

Reads:

- RealSense depth;
- RealSense tags;
- bump sensors;
- depth angles;
- valid depth mask.

The output structure is consumed by `scoreParticlesWithBeacons` and
`scoreParticlesWithDepth`.

### `getInitializationOdometry.m`

Reads Create odometry during initialization.

Output convention:

```matlab
odom.d
odom.phi
odom.valid
```

### `checkInitializationConvergence.m`

Convergence check for initialization.

It does not only look at max weight. It combines:

- dominant waypoint probability;
- position covariance;
- circular heading standard deviation;
- stability over several updates.

### `computeWaypointPosterior.m`

Aggregates particle weights by candidate waypoint.

Output is a probability mass for each possible starting waypoint.

### `estimatePoseFromParticles.m`

Estimates pose from particles and weights.

During initialization, this estimates pose from the dominant waypoint cluster
instead of averaging all hypotheses together.

### `propagateParticles.m`

PF motion model.

Input is a particle set and odometry increment. Output is the propagated
particle set.

Used by both initial localization and PF waypoint following.

### `scoreParticlesWithBeacons.m`

Computes beacon log-likelihood for each particle.

Features:

- Uses tag ID for data association.
- Supports camera-frame `[xCam, yCam]` observations.
- Also supports range/bearing-style observations.

### `scoreParticlesWithDepth.m`

Computes depth log-likelihood for each particle using known-map ray casting.

Important:

- Uses only the provided `map` / `verifiedMap`.
- Does not automatically use unknown optional walls.

### `combineParticleLogWeights.m`

Combines previous weights, beacon likelihood, and depth likelihood in log-space.

This avoids numerical underflow from multiplying many small probabilities.

### `systematicResample.m`

Systematic resampling for particle filters.

Used by both initial localization and PF waypoint following.

### `circularStdWeighted.m`

Weighted circular standard deviation.

Used for heading uncertainty, where normal linear averaging would be wrong near
the angle wrap-around boundary.

## EKF Waypoint Following

### `runEkfWaypointFollower.m`

Main EKF localization plus waypoint-following function.

Interface:

```matlab
[ekfState, dataStore, navState] = runEkfWaypointFollower( ...
    Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, initState, ekfParams, dataStore)
```

Inputs:

- `map`: current known / verified map.
- `goalWaypoints`: planner-generated path points, not necessarily all scoring
  waypoints.
- `initState`: result from initial localization.
- `ekfParams`: EKF and controller parameters.

Main loop:

1. Read odometry, depth, beacon, and bump sensors.
2. EKF prediction.
3. Beacon correction.
4. Depth correction.
5. Compute waypoint command from EKF pose.
6. Beep when a scoring waypoint is reached.
7. Run temporary bump recovery if needed.
8. Log to `dataStore.ekf...`.

`ekfState` includes:

- `pose`
- `poseCov`
- `reachedAllGoals`
- `stopReason`
- `numUpdates`
- `runTimeSec`
- `params`

### `ekfWaypointDefaultParams.m`

Default parameters for EKF waypoint following.

Includes:

- runtime;
- controller parameters;
- velocity limits;
- EKF process noise;
- beacon update noise/gating;
- depth update noise/gating;
- bump recovery parameters;
- live plot parameters.

Important parameters:

```matlab
params.closeEnough
params.maxFwdVel
params.maxAngVel
params.recoveryBackDistance
params.recoveryTurnAngle
params.enableLivePlot
```

### `GjacDiffDrive.m`

Jacobian for the differential-drive motion model.

Used during EKF prediction to update covariance.

### `integrateOdom.m`

Integrates a robot pose using an odometry increment.

Used by EKF prediction and recovery-state updates.

## PF Waypoint Following

### `runPfWaypointFollower.m`

Main PF localization plus waypoint-following function.

Interface:

```matlab
[pfState, dataStore, navState] = runPfWaypointFollower( ...
    Robot, map, beaconLoc, goalWaypoints, offset_x, offset_y, initState, pfParams, dataStore)
```

Goals:

- Exist in parallel with `runEkfWaypointFollower`.
- Avoid replacing the current integrated EKF flow.
- Test whether PF tracking is more stable during waypoint following.

Main loop:

1. Inherit particles and weights from `initState.particles/weights`.
2. Read odometry, depth, beacon, and bump sensors.
3. Propagate particles with `propagateParticles`.
4. Update weights with beacon and depth likelihoods.
5. Resample when ESS is low.
6. Estimate current pose and covariance from weighted particles.
7. Track waypoints using that estimated pose.
8. Beep at scoring waypoints.
9. On bump, back up, turn, and replan to the current target.
10. Log to `dataStore.pf...`.

`pfState` includes:

- `pose`
- `poseCov`
- `particles`
- `weights`
- `reachedAllGoals`
- `stopReason`
- `numUpdates`
- `runTimeSec`
- `params`

### `pfWaypointDefaultParams.m`

Default parameters for PF waypoint following.

It inherits many defaults from `ekfWaypointDefaultParams` and
`initDefaultParams`.

Includes:

- controller parameters;
- particle count;
- local initialization standard deviations;
- PF process noise;
- beacon/depth likelihood parameters;
- resampling parameters;
- bump recovery parameters;
- live plot parameters.

### `plotPfWaypointResult.m`

Offline plot for PF waypoint-following results.

Typical usage:

```matlab
plotPfWaypointResult('latestPfWaypointPracticeRun.mat')
```

Shows:

- known map;
- beacons;
- waypoints;
- goal sequence;
- PF estimated path;
- final particles;
- final covariance;
- final heading.

## Path Planning

### `knownMapPlannerDefaultParams.m`

Default parameters for the known-map planner.

Controls:

- wall clearance;
- roadmap sampling;
- grid sampling;
- obstacle inflation;
- search behavior.

### `planPathKnownMap.m`

Plans a collision-free path from the current point to one goal.

Method:

1. Build roadmap nodes.
2. Include start, goal, wall-corner samples, and sparse grid samples.
3. Add an edge if the straight segment does not cross a wall and has enough
   clearance.
4. Run Dijkstra to find the shortest path.

Outputs:

- `path`
- `pathInfo`

### `planWaypointSequenceKnownMap.m`

Plans through multiple high-level waypoints.

It calls `planPathKnownMap` for each high-level goal and concatenates the
segments.

It also produces `beepMask`:

- intermediate planner nodes do not beep;
- real high-level waypoints beep.

### `plotKnownMapPath.m`

Offline plot for debugging the known-map planner.

Shows known map, beacons, waypoints, and planned path.

## Optional Wall Verification

### `verifyOptionalWalls.m`

Main active optional-wall verification function.

Interface:

```matlab
[wallStatus, dataStore, verifiedMap] = verifyOptionalWalls( ...
    Robot, map, optWalls, beaconLoc, offset_x, offset_y, initState, ekfParams, plannerParams, verifyParams, dataStore)
```

Core logic:

1. Generate an observation point for each optional wall.
2. Plan to the observation point using the current map.
3. Turn toward the wall midpoint.
4. Collect depth data.
5. Compare residuals under the absent-wall and present-wall models.
6. Run a low-speed bump probe when needed.
7. Classify the wall as `exists`, `absent`, or `unknown`.
8. If the wall exists, add it to `verifiedMap`.

The current classifier uses both:

- depth residuals;
- bump probe contact / no-contact evidence.

When `verifyParams.followerMode = 'pf'`, this function uses
`runPfWaypointFollower` to drive to observation/probe points. The default mode is
still EKF, so existing EKF optional-wall tests keep the same behavior.

### `optionalWallDefaultParams.m`

Default parameters for optional-wall verification.

Includes:

- observation-point search parameters;
- depth classification parameters;
- bump-probe parameters;
- fallback bump-probe parameters;
- debug print switch.

Important parameters:

```matlab
params.bumpProbeBackupDistance
params.bumpProbeSpeed
params.bumpProbeMaxTravel
params.enableBumpProbeFallback
```

### `plotOptionalWallVerificationResult.m`

Offline plot for optional-wall verification.

Shows:

- known map;
- optional-wall classification results;
- observation points;
- waypoints;
- beacons;
- EKF trajectory.

### `runOptionalWallHeadlessSmoke.m`

Headless smoke test for optional-wall verification.

Useful for rough validation without going through the full GUI workflow.

## Plotting And Visualization

### `plotInitializationResult.m`

Plot for initial localization.

Shows:

- known map;
- beacons;
- candidate waypoints;
- particles;
- estimated pose;
- selected start.

### `plotEkfWaypointResult.m`

Offline plot for EKF waypoint-following results.

Typical usage:

```matlab
plotEkfWaypointResult('latestEkfWaypointPracticeRun.mat')
```

Shows:

- known map;
- beacons;
- waypoints;
- goal sequence;
- EKF trajectory;
- final covariance;
- final heading.

### `plotPfWaypointResult.m`

Offline plot for PF waypoint-following results.

See the PF waypoint-following section.

### Live Plots

Both EKF and PF followers support live plotting:

- EKF window name: `EKF Waypoint Live`
- PF window name: `PF Waypoint Live`

Switch:

```matlab
params.enableLivePlot = true;
```

## Robot / Simulator Wrappers

### `stopRobotSafe.m`

Safe stop wrapper.

Internally calls:

```matlab
SetFwdVelAngVelCreate(Robot, 0, 0)
```

It does not try to send movement commands through `Robot.CreatePort`.

### `getCreateSensorPort.m`

Returns the Create port needed by some sensor-reading functions.

Important distinction:

- movement commands use `Robot`;
- some course sensor functions may need a CreatePort-like handle, so sensor
  readers use this helper for compatibility.

### `readStoreSensorData.m`

Course-style sensor logging helper.

This comes from the older backup/template style. The current main flows mostly
use each follower's own sensor-read and logging logic.

### `backupBump.m`

Early template/example code.

Useful as a reference for:

- using `global dataStore`;
- reading and logging sensors;
- handling bump sensors;
- writing an Autonomous Start entry.

The current main system does not directly depend on it.

## Math, Geometry, And Sensor Helpers

### `wrapToPiLocal.m`

Local angle wrapping function.

Normalizes angles around the `[-pi, pi)` range.

### `global2robot.m`

Transforms a global-frame point into the robot frame.

Used by beacon prediction.

### `predictBeaconObservations.m`

Predicts beacon observations from a pose/particle and global beacon locations.

Used by the PF beacon likelihood.

### `precomputeKnownMapGeometry.m`

Precomputes wall geometry for ray casting.

### `raycastKnownMap.m`

Ray casts against the known / verified map.

Input is pose, beam angles, and sensor offset. Output is predicted depth.

Used by:

- initialization PF depth scoring;
- EKF depth update;
- PF waypoint depth scoring;
- optional-wall depth classification.

### `feedbackLin.m`

Feedback-linearization controller.

Converts desired global-frame velocity `(cmdVx, cmdVy)` into robot command:

```matlab
[cmdV, cmdW]
```

### `limitCmds.m`

Scales `cmdV/cmdW` to respect wheel-speed limits.

## Data Files And Latest Runs

### `PracticeMap2026.mat`

Current MATLAB practice map.

Test entries load:

- `map`
- `optWalls`
- `waypoints`
- `beaconLoc`

### `PracticeMap2026.txt`

Text version of the practice map for human inspection.

### `latestInitLocalizationPracticeRun.mat`

Most recent initial-localization test result.

### `latestEkfWaypointPracticeRun.mat`

Most recent EKF waypoint-following test result.

### `latestPfWaypointPracticeRun.mat`

Most recent PF waypoint-following test result.

### `latestOptionalWallPracticeRun.mat`

Most recent optional-wall verification test result.

### `latestIntegratedPracticeRun.mat`

Most recent integrated waypoint-plus-optional-wall test result.

These `latest*.mat` files are run logs. Core code should not depend on them.

## Main Data Flows

### EKF waypoint test flow

```text
testEkfWaypointFollowingPracticeMap
  -> initializeLocalization
  -> planWaypointSequenceKnownMap
  -> runEkfWaypointFollower
  -> latestEkfWaypointPracticeRun.mat
  -> plotEkfWaypointResult
```

### PF waypoint test flow

```text
testPfWaypointFollowingPracticeMap
  -> initializeLocalization
  -> planWaypointSequenceKnownMap
  -> runPfWaypointFollower
  -> latestPfWaypointPracticeRun.mat
  -> plotPfWaypointResult
```

### PF waypoint plus optional-wall test flow

```text
testPfWaypointOptionalPracticeMap
  -> initializeLocalization
  -> planWaypointSequenceKnownMap
  -> runPfWaypointFollower
  -> verifyOptionalWalls with followerMode='pf'
  -> latestPfWaypointOptionalPracticeRun.mat
```

### PF normal waypoint, optional-wall, plus EC waypoint test flow

```text
testPfWaypointEcOptionalPracticeMap
  -> initializeLocalization
  -> planWaypointSequenceKnownMap for normal waypoints
  -> runPfWaypointFollower
  -> verifyOptionalWalls with followerMode='pf'
  -> EC path planning in map-file order on verifiedMap
  -> runPfWaypointFollower with verifiedMap
  -> latestPfWaypointEcOptionalPracticeRun.mat
```

### Integrated test flow

```text
testIntegratedWaypointOptionalPracticeMap
  -> initializeLocalization
  -> planPathKnownMap
  -> selectRelevantOptionalWalls
  -> verifyOptionalWalls when needed
  -> runEkfWaypointFollower
  -> post-waypoint optional-wall verification
  -> latestIntegratedPracticeRun.mat
```

## Current Design Choices

- Initial localization uses PF.
- Main waypoint following currently uses EKF.
- PF waypoint following is a parallel experimental interface, not a replacement
  for EKF.
- Optional-wall verification still uses the EKF follower to reach observation
  and probe points.
- Initialization does not use optional walls.
- The depth model only uses the provided known / verified map.
- Intermediate planner nodes do not beep; real high-level waypoints beep.
- Normal-navigation bump recovery uses `recoveryBackDistance`.
- Optional-wall contact probing uses `bumpProbeBackupDistance`.
