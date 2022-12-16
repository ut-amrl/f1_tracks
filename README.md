# f1_tracks

A dataset of F1 racetracks used for F1/10 racing. Created using original F1 tracks from [TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database).

To get the scans, buld and run the `trackpoints_to_scans` binary. The scans will be generated under `scans/<track_name>.scans.csv` by default.
The format of the CSV files is such that every row is composed of an (x, y, theta) pose followed by each range measurement in the scan.

## Dependencies
- 'Eigen'
- `glog`
- `gflags`
- `Boost`'s filesystem component

