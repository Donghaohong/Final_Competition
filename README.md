# FinalCompetition Project Extract

This folder is a clean extraction of the current final competition code.

## Layout

- `code/`: MATLAB code for initialization localization, EKF waypoint following, known-map path planning, optional-wall verification, plotting, wrappers, and test entries.
- `maps/`: Practice map files copied from the working `Code` folder.
- `docs/`: Competition PDFs and pitch document.

`PracticeMap2026.mat` and `PracticeMap2026.txt` are also copied into `code/` because the current test entry functions load the map from their own folder.

## Main Entries

- `testInitializeLocalizationPracticeMap(Robot)`
- `testEkfWaypointFollowingPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointFollowingPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointOptionalPracticeMap(Robot, goalWaypoints)`
- `testPfWaypointEcOptionalPracticeMap(Robot, goalWaypoints, ecGoalWaypoints)`
- `testOptionalWallVerificationPracticeMap(Robot)`
- `testIntegratedWaypointOptionalPracticeMap(Robot, goalWaypoints)`

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
