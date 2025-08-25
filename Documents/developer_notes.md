# Developer Notes

## Calibration Sensors

### variance_cv_mask
Reports the coefficient of variation for the MCPS mask collected during a passive scan. Higher values indicate inconsistent readings across the region-of-interest grid.

### trial_bump_cv
Represents the coefficient of variation for distance measurements observed in each trial of a passive scan. It helps evaluate the stability of bump detection across trials.

### scan_time_cap_seconds
Publishes the total elapsed time of the passive scan session in seconds. Scanning stops before the 16x16 grid when this elapsed time exceeds the configured cap.
