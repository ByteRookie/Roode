# Developer Notes

## Calibration Sensors

### variance_cv_mask
Reports the coefficient of variation for the MCPS mask collected during a passive scan. Higher values indicate inconsistent readings across the region-of-interest grid.

### trial_bump_cv
Represents the coefficient of variation for distance measurements observed in each trial of a passive scan. It helps evaluate the stability of bump detection across trials.

### scan_time_cap_seconds
Publishes the total elapsed time of the passive scan session in seconds. When a
`scan_time_cap_seconds` value is configured, the passive scan stops before
running the 16x16 grid once the elapsed time exceeds this cap.
