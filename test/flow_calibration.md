# Optical flow bench tests

Bench tests are currently compiled out for normal operation. To restore them,
set ENABLE_BENCH_TESTS to 1 in include/bench_config.h and rebuild/upload.

After any flow test ends (including X or timeout), pre-round serial output
pauses so the summary stays visible. Send **C** to resume monitoring and re-arm
the start button, or **P/F/O** to run another test directly. X also pauses output
when already waiting between tests. Serial commands remain available; G can
still start a round. This pause does not apply during a running round.

## Raw-flow troubleshooting — send P

Before the round starts, send **P**. Motors remain off. Wait for
`FLOW_BEGIN,test=PUSH`, then push the robot straight forward a measured 500 mm
without lifting it or rotating it. Stop moving and send **X** to finish (or
wait for the 30-second limit). Paste FLOW_BEGIN, FLOW_RAW, and FLOW_SUMMARY lines
with the measured displacement. X is normal completion for this manual test.

`sum_x`/`sum_y` accumulate every raw sensor read; `abs_x`/`abs_y` accumulate
absolute counts to reveal cancellation. Individual `raw_x`/`raw_y` fields are
only the most recent sample at each print. `sensor_forward_mm`/`sensor_left_mm`
are summed body-axis readings before compensation. `correction_*` shows the
summed rotation correction, and `pose_*` shows final displacement relative to
the starting heading. Sensor/correction sums are in the changing body frame;
compare them directly with pose only for nearly straight runs.

`zero_samples` includes time stationary. `integrated_samples` counts pose updates
that applied flow; a failed heading read can defer integration. Raw reads do not
prove valid optical tracking or SPI communication; the installed motion-read API
does not supply those quality checks here.

For a controlled comparison, repeat the same measured push on the arena floor
with the robot LEDs on and off, keeping other lighting unchanged. A further
comparison on a matte patterned surface at the same lens height can help isolate
surface/lighting sensitivity. These comparisons do not assume either is the cause.

Upload the firmware, close the visualiser (it owns the serial port), and open a
serial monitor at 115200 baud. Leave the robot waiting for the start button;
do not send G or start a round. ENABLE_BENCH_TESTS must be 1.

These tests run the real pose integration and rotation compensation, without
mapping or wall anchoring. They drive the robot: use clear floor space, keep the
USB cable slack, and send **X** to stop. Obstacle avoidance and collection are
not running during these isolated tests. Each leg has a 15-second timeout and
stops on a failed heading read. Flow readiness only confirms initialization;
it cannot detect all poor-tracking conditions.

## 1. Distance scale — send F

1. Use the competition floor surface and final mounting height (~104 mm).
2. Mark a fixed point on the chassis and its initial position on the floor.
3. Send **F**. After two seconds the robot drives forward until its compensated
   pose reports 500 mm, then stops and measures another 700 ms of settling.
4. Measure the actual displacement of that same chassis point along the initial
   heading, including coasting. Also note physical sideways movement and yaw.
5. Copy the `FLOW_BEGIN` and `FLOW_SUMMARY` lines, plus your tape measurement.
   Repeat at least three times, repositioning between trials.

Compare the tape measurement with **forward_mm**, not the nominal 500 mm:
`stop_forward_mm` is the distance when stop was commanded; `forward_mm` includes
coasting. `left_mm` and `max_heading_error_deg` help identify curved runs.
Use straight runs for scale calibration.

    new metres/count = printed metres/count * actual forward mm / forward_mm

For example, 550 mm physically versus 500 mm reported means under-reporting:
increase the scale by 10%. The current estimated scale at 104 mm is 0.0002184
metres/count. Set FLOW_METERS_PER_COUNT in src/pose.cpp to the measured value,
rebuild, and repeat. Do not adjust sensor offsets to fix a distance scale error.

## 2. Rotation compensation — send O (letter O)

Mark the robot's turning centre on the floor and record video from above if
possible. Send **O**: after two seconds it turns approximately 360 degrees CCW,
stops for 700 ms, then does the same CW. Each direction prints its own summary.
The actual `turn_deg` includes coasting, so some overshoot is expected.

Copy both summaries and preferably all `FLOW_SAMPLE` lines. Report whether the
physical turning centre stayed put or shifted, and by roughly how much.

- `final_displacement_mm`: final reported distance from the leg's start.
- `peak_displacement_mm`: greatest reported distance from that start at any
  sample, including settling. A full-circle error can cancel at the end, so
  this matters even if final displacement is tiny.
- `pose_path_mm`: sum of reported translation between samples; sensitive to
  noise, and not a direct measurement of physical drift.
- `turn_deg`: accumulated heading change, retaining full turns.
- `forward_mm` / `left_mm`: final translation relative to the starting heading.

Calibrate scale first, then assess rotation. Real chassis translation should
appear in the pose; it is not necessarily compensation error. If the centre
stays still but the pose traces an arc, check the sensor axis signs and measured
offsets in src/pose.cpp. Current offsets are -210.5 mm forward and -138.25 mm
left (behind and right of the turning centre). These are horizontal mounting
offsets, separate from lens-to-floor height. Repeated direction-dependent errors
can also indicate timing or tracking effects; paste the results before changing
offsets blindly.

Results marked ABORTED, TIMEOUT, or HEADING_FAILURE are diagnostic only; do not
use them for scale calibration. The test does not automatically change calibration
or declare accuracy without an external measurement. Reset before a real round.
