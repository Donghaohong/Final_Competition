# FinalCompetition Project Extract

This folder is a clean extraction of the current final competition code.

## Layout

- `code/`: MATLAB code for initialization localization, EKF waypoint following, known-map path planning, optional-wall verification, plotting, wrappers, and test entries.
- `maps/`: Practice map files copied from the working `Code` folder.
- `docs/`: Competition PDFs and pitch document.

`PracticeMap2026.mat` and `PracticeMap2026.txt` are also copied into `code/` because the current test entry functions load the map from their own folder.

## Main Entries

- `finalCompetition(Robot, maxTime, offset_x, offset_y)`
- `testInitializeLocalizationPracticeMap(Robot)`
- `testEkfWaypointFollowingPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointFollowingPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointOptionalPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointEcOptionalPracticeMap(Robot, goalWaypoints, ecGoalWaypoints)`
- `testOptionalWallVerificationPracticeMap(Robot)`
- `testIntegratedWaypointOptionalPracticeMap(Robot, goalWaypoints)`

## Final Competition Entry

`code/finalCompetition.m` is the current official-style entry point. It runs:

1. particle-filter initialization localization;
2. EKF normal waypoint following;
3. EKF optional-wall verification;
4. EKF EC waypoint following on the updated `verifiedMap`;
5. final result save and plot.

The current development version intentionally loads `PracticeMap2026.mat`
before `compMap.mat` so PracticeMap testing remains reproducible. Before the
actual competition submission, switch the loader order back to `compMap.mat`
first.

`finalCompetition` mirrors returned data into a global `dataStore` and saves
`code/latestFinalCompetitionRun.mat`. It also writes
`code/latestFinalCompetitionResult.fig`, showing the map, EKF trajectory,
visited scoring waypoints, and optional walls with the required display rules:
known walls black, existing optional walls black, unknown optional walls red,
and absent optional walls hidden.

## Current Integrated Flow

`testIntegratedWaypointOptionalPracticeMap` is the current top-level PracticeMap
test entry. It runs:

1. particle-filter initialization localization;
2. EKF localization with known-map waypoint following;
3. optional-wall checks when a planned path is relevant to an unknown wall;
4. post-waypoint optional-wall verification for any walls still marked unknown.

Navigation bump recovery uses `ekfWaypointDefaultParams.recoveryBackDistance`.
Optional-wall contact probing uses
`optionalWallDefaultParams.bumpProbeBackupDistance`.

The current EKF waypoint speed settings are conservative and below the
competition limit: `vxyMax = 0.13`, `maxWheelSpeed = 0.13`,
`maxFwdVel = 0.13`, and `wheel2Center = 0.13`. `wheel2Center` is the robot
half-wheelbase geometry parameter, not a speed.

`testPfWaypointFollowingPracticeMap` is a parallel particle-filter waypoint
following test entry for A/B comparison against the EKF follower. It does not
replace the integrated EKF flow.

`testPfWaypointOptionalPracticeMap` uses PF for waypoint following and also sets
optional-wall verification to use the PF follower when driving to observation or
probe points.

`testPfWaypointEcOptionalPracticeMap` runs PF through normal waypoints first,
then verifies optional walls with the PF follower, and finally visits EC
waypoints in `ECwaypoints` file order using paths planned on the updated
`verifiedMap`.

## Notes

- The extracted code is based on dependency analysis from the current project entry points.
- Old homework/lab files, generated latest-run logs, and unrelated `.mat` data were intentionally not copied.
