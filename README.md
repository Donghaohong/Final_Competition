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
- `testOptionalWallVerificationPracticeMap(Robot)`

## Notes

- The extracted code is based on dependency analysis from the current project entry points.
- Old homework/lab files, generated latest-run logs, and unrelated `.mat` data were intentionally not copied.
