# GuardROS Experimental Protocol
## IEEE Robotics and Automation Practice (RA-P)

---

# 1. Experimental Environment

## Hardware
- GuardBot RollerBot 23
- Original robot onboard hardware preserved
- Raspberry Pi onboard computer
- Laptop running Ubuntu 22.04 LTS
- ROS 2 Humble

## Communication
- Wi-Fi network
- Original robot firmware preserved
- No onboard firmware modifications

## Software
- GuardROS framework
- ROS 2 Humble
- Python-based experiment logger
- Python offline statistical analysis pipeline

---

# 2. Experimental Data Collection

The following data streams are continuously logged:

- cmd_vel commands
- telemetry JSON packets
- typed telemetry variables
- video packet timing
- audio packet timing
- driver diagnostics
- UDP communication diagnostics

---

# 3. Offline Analysis

The offline analysis pipeline computes:

- mean
- median
- standard deviation
- P95
- P99
- timing jitter
- communication gaps
- packet rates
- derived communication metrics

The analysis pipeline also generates:

- timing timelines
- timing histograms
- rate-over-time plots
- battery plots
- temperature plots
- network latency plots

---

# 4. Official Experimental Runs

---

# T0 — Pilot Official Run

## Objective
Validate the complete experimental pipeline before official data acquisition.

## Duration
10 min

## Conditions
- video enabled
- telemetry enabled
- moderate teleoperation
- experiment logger enabled

## Expected Outcome
- valid logs
- valid plots
- absence of UDP communication errors

---

# T1 — Idle Stability Test

## Objective
Evaluate communication and streaming stability while the robot remains stationary.

## Duration
30 min

## Conditions
- stationary robot
- video enabled
- telemetry enabled
- audio RX enabled
- experiment logger enabled

## Main Metrics
- FPS stability
- telemetry rate stability
- UDP timeout count
- CPU temperature
- timing jitter

---

# T2 — Teleoperation Test

## Objective
Evaluate system behavior during manual teleoperation.

## Duration
15 min

## Standardized Motion Sequence
Continuously repeat:
- forward motion
- backward motion
- left rotation
- right rotation
- combined motion

## Main Metrics
- cmd_vel rate stability
- telemetry continuity
- video continuity
- command jitter

---

# T3 — Video Stress Test

## Objective
Evaluate video streaming robustness under continuous robot motion.

## Duration
15 min

## Conditions
- continuous motion
- frequent direction changes
- video continuously enabled

## Main Metrics
- FPS
- frame timing jitter
- non-JPEG packet count
- UDP anomalies

---

# T4 — Audio Stress Test

## Objective
Evaluate bidirectional audio streaming robustness.

## Duration
10 min

## Conditions
- RX audio enabled
- TX audio enabled
- intermittent speech

## Main Metrics
- audio RX rate
- audio TX rate
- UDP packet stability

---

# T5 — Long Duration Stability Test

## Objective
Evaluate long-term operational stability.

## Duration
60 min

## Conditions
- video enabled
- telemetry enabled
- experiment logger enabled
- periodic light teleoperation

## Main Metrics
- sustained FPS
- sustained telemetry rate
- thermal behavior
- cumulative packet errors
- reconnection issues

---

# T6 — Proprietary Application Comparison

## Objective
Compare GuardROS with the original GuardBot application.

## Type
Qualitative and operational comparison.

## Evaluated Aspects
- startup procedure
- ROS integration
- logging capability
- extensibility
- telemetry accessibility
- automation capability

---

# 5. Official Dataset Organization

Official runs shall be organized as:

experiments/official_runs/

- T0_pilot/
- T1_idle/
- T2_teleoperation/
- T3_video_stress/
- T4_audio_stress/
- T5_long_run/
- T6_guardbot_app_comparison/

---

# 6. Notes

- All experiments are performed using the original robot firmware.
- No firmware modification is performed onboard the robot.
- GuardROS operates as an external ROS-based integration framework.
- The experimental pipeline is fully reproducible.