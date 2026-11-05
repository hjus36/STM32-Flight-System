# Pitch Double PID - Simulation Validation Report

- Model: `Pitch_Double_PID.slx`
- Date: 2026-11-04 15:36:27
- Simulation window: 0.000 to 10.000 s
- Measured median sample interval: 0.010000 s
- Step at t = 1.0000 s, target = 5.000000 deg

## Controller and plant assumptions

- Outer loop: angle P = 2 (configured learning model)
- Inner loop: rate PI, Kp = 0.05, Ki = 0.2 (configured learning model)
- Assumed plant: Jy = 0.002 kg*m^2, By = 0.02 N*m*s/rad
- The script extracts simulation outputs; these nominal parameter settings are not independently read from Simulink block dialogs.

## Measured results

| Metric | Value |
|---|---:|
| Angle at stop | 5.00000000 deg |
| Error at stop | 0.00000000 deg |
| Rise time (10% to 90% of commanded step) | 0.950000 s |
| Settling time (target +/-2% of step amplitude) | 1.550000 s |
| Overshoot over target | 0.102426 % |
| Peak absolute actual angular rate | 0.13234099 rad/s |
| Peak absolute PI control torque | 0.00872665 N*m |
| IAE from step to stop | 2.48510066 deg*s |
| Integral of squared torque from step to stop | 3.955016e-06 (N*m)^2*s |

## Metric definitions

- Rise time: elapsed time between first sampled crossings of 10% and 90% of the commanded angle step.
- Settling time: elapsed time since command step after which all remaining recorded samples stay within +/-2% of the commanded step amplitude.
- `NaN` means the criterion was not reached before the simulation stopped.
- Error at stop is not necessarily infinite-time steady-state error.
- IAE = time integral of the absolute target-angle tracking error from step to stop.

## Files

- `pitch_double_pid_signals.csv` - time-aligned logged signals
- `pitch_double_pid_summary.csv` - measured metrics
- `pitch_double_pid_simulation_data.mat` - MATLAB analysis data
- `figures_clean/` - four exported plots

## Roll vs Pitch comparison (optional)

- Reference: `../roll_double_pid_sim/analysis/roll_double_pid_signals.csv`
- Max absolute actual-angle difference: 5.551115123e-17 rad
- Max absolute actual-rate difference: 4.718447855e-16 rad/s
- Max absolute control-torque difference: 4.987329993e-18 N*m
- Differences here reflect sampled simulation signals; identical parameters should yield equivalent response.
- See `roll_vs_pitch_comparison.csv` and `figures_clean/05_roll_vs_pitch_angle_clean.png`.

## Validation boundary

- These results validate the simplified simulated pitch cascade under the specified step test.
- Drone inertia, motor/ESC dynamics, sensors, saturation, disturbance rejection, MCU timing, and real flight behavior are NOT validated.
- No STM32 code generation, arm build, or deployment is performed by this analysis script.
