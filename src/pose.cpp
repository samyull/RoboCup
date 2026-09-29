#include "pose.h"
#include <Wire.h>
#include <SPI.h>
#include "motors.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Bitcraze_PMW3901.h>

// ---------------------------------------------------------------------------
// Flow sensor calibration / mounting
// ---------------------------------------------------------------------------


// Lens-to-floor height in metres. See pose.h - CALIBRATE FLOW_METERS_PER_COUNT
// with a known-distance push test once this is set.
// Raised 24 mm from 80 mm to ~104 mm (estimate - measure lens to floor to confirm).
float MOUNTING_HEIGHT = 0.104F;
// Calibrated with F tests (original lighting, 70% power):
//  1. At 0.0002184 (height estimate): 500 -> 585 mm, 415 -> ~500 mm -> 0.000250.
//  2. At 0.000250: 500 -> 530 mm taped (coast estimated) -> 0.000256.
//  3. At 0.000256: 460 mm taped vs 514 mm pose displacement -> 0.000229 (used).
// Run 3 disagrees with 1-2 by ~10% although tracking was clean (SQUAL 97-112),
// so confirm with a few more F runs. new = this * actual_mm / final_displacement_mm.
float FLOW_METERS_PER_COUNT = 0.000229f;   // 0.229 mm/count

// Map the sensor's raw dx/dy onto the robot body frame (x = forward, y = left).
// Push the robot forward, then to the left, and watch raw_dx / raw_dy in the
// debug print. Set these so pushing forward makes x increase and pushing
// left makes y increase.
static const bool  FLOW_SWAP_XY   = true;  // true if the sensor's dy is the forward axis
static const float FLOW_SIGN_FWD  = -1.0f;   // flip to -1 if forward push gives negative x
static const float FLOW_SIGN_LEFT = -1.0f;   // flip to -1 if left push gives negative y

// The sensor is mounted slightly rotated: driving dead straight (checked against a
// tape line) it reported ~4.4 deg of sideways movement to the left (F tests:
// 4.19/4.56/4.55 deg). Positive = reports drift LEFT when driving straight.
// Tune so an F test gives left_mm ~ 0 on a run that is physically straight.
// TEMPORARILY 0 for the original-lighting baseline test. Measured value: 4.43f.
static const float FLOW_MOUNT_ANGLE_DEG = 0.0f;

// Flow sensor position relative to the robot's centre of rotation, in metres
// (forward, left). Leave at 0 if it sits over the centre. If it doesn't,
// spinning the robot makes it report fake translation, which is removed below.
static const float FLOW_OFFSET_FWD_M  = -0.2105f;
static const float FLOW_OFFSET_LEFT_M = -0.13825f;


// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static const uint8_t IMU_ADDRESS = 0x28;
static const uint32_t HEADING_TIMEOUT_MS = 200;
static Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDRESS, &Wire1);
static Bitcraze_PMW3901 *flow_sensor = nullptr;
#if ENABLE_BENCH_TESTS
static uint8_t flow_cs_pin = 0;  // for the tracking-quality register reads
#endif

static Pose current_pose;
static float theta_offset = 0.0f; // counter-clockwise IMU heading at pose_reset(), subtracted out
static bool imu_ready = false;
static bool flow_ready = false;

// Last raw readings, kept around purely so pose_print_debug() can show them.
static float last_heading_deg_raw = 0.0f;
static int16_t last_dx_counts = 0;
static int16_t last_dy_counts = 0;
static bool last_euler_read_ok = false;

static uint32_t last_heading_sample_us = 0;
static bool have_heading_sample_time = false;
static bool discard_stationary_flow = false;
static uint32_t last_heading_sample_ms = 0;
static float pending_flow_fwd_m = 0.0f;
static float pending_flow_left_m = 0.0f;

// Flow still counts for this long after the motors are commanded to 0 (roll-on).
static const uint32_t STOPPED_GRACE_MS = 400;
static bool stationary_filter = true;
static bool flow_blind = false;
static float blind_skipped_x = 0.0f, blind_skipped_y = 0.0f;  // world frame, m

// getEvent() does not propagate I2C failures in the installed library.
// Read the little-endian heading directly, using the same degree scale.
static bool read_heading(float &heading_deg) {
  Wire1.beginTransmission(IMU_ADDRESS);
  Wire1.write((uint8_t)Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR);
  if (Wire1.endTransmission(false) != 0) return false;
  const uint8_t received = Wire1.requestFrom(IMU_ADDRESS, (uint8_t)2);
  if (received != 2 || Wire1.available() != 2) {
    while (Wire1.available()) Wire1.read();
    return false;
  }
  const uint8_t lo = Wire1.read();
  const uint8_t hi = Wire1.read();
  const uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  if (raw >= 360U * 16U) return false;
  heading_deg = raw / 16.0f;
  return true;
}

/* Wrap an angle in radians to [-pi, pi]. */
static float wrap_angle(float angle_rad) {
  while (angle_rad > PI)  angle_rad -= 2.0f * PI;
  while (angle_rad < -PI) angle_rad += 2.0f * PI;
  return angle_rad;
}

static float last_angular_speed = 0.0f;


// ---------------------------------------------------------------------------
// Init / reset (init sequence unchanged from the version that worked)
// ---------------------------------------------------------------------------


void pose_init(uint8_t flow_chip_select) {
  Wire.begin();
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected - check wiring/address");
    imu_ready = false;
  } else {
    delay(1000); // let fusion fully settle after begin()'s own internal NDOF setup
    // NOTE: setExtCrystalUse(true) removed - many BNO055 breakout boards
    // don't have the external crystal populated, and telling the chip to
    // use a clock source that doesn't exist can prevent sensor fusion from
    // starting. Add it back only if your board's datasheet confirms it has
    // one fitted.
    // NOTE: explicit setMode(OPERATION_MODE_NDOF) removed too - begin()
    // already sets NDOF mode internally, and writing it again immediately
    // afterward was colliding with that internal write (system_error=6,
    // "register map write error").
    imu_ready = true;
    Serial.println("BNO055 initialised");
  }

#if ENABLE_BENCH_TESTS
  flow_cs_pin = flow_chip_select;
#endif
  flow_sensor = new Bitcraze_PMW3901(flow_chip_select);
  if (!flow_sensor->begin()) {
    Serial.println("ERROR: PMW3901 not detected - check wiring/CS pin");
    flow_ready = false;
  } else {
    flow_ready = true;
    Serial.println("PMW3901 initialised");
  }

  if (imu_ready) {
    // Give fusion a moment before trusting status/output.
    delay(1000);
    uint8_t system_status, self_test_result, system_error;
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);
    Serial.print("BNO055 system_status="); Serial.print(system_status);
    Serial.print(" (0=idle,1=error,2=init-periph,3=init-system,4=self-test,5=fusion-running,6=no-fusion-running)");
    Serial.print(" self_test_result=0x"); Serial.print(self_test_result, HEX);
    Serial.print(" (bit0=accel,bit1=mag,bit2=gyro,bit3=MCU - 0xF means all passed)");
    Serial.print(" system_error="); Serial.println(system_error);
  }

  pose_reset();
}

// ---------------------------------------------------------------------------
// Pose history - lets a ToF reading be placed using the pose at the moment it
// was measured rather than when it was read out (matters while turning).
// ---------------------------------------------------------------------------

static const int POSE_HISTORY_LEN = 32;  // ~1 s at the loop rate; readings are < 0.2 s old
struct PoseSample {
  uint32_t ms;
  Pose pose;
};
static PoseSample pose_history[POSE_HISTORY_LEN];
static int pose_history_count = 0;
static int pose_history_next = 0;  // slot the next sample is written to

static void clear_pose_history() {
  pose_history_count = 0;
  pose_history_next = 0;
}

static void record_pose_sample(uint32_t ms, const Pose &pose) {
  pose_history[pose_history_next] = {ms, pose};
  pose_history_next = (pose_history_next + 1) % POSE_HISTORY_LEN;
  if (pose_history_count < POSE_HISTORY_LEN) ++pose_history_count;
}

bool pose_at(uint32_t t_ms, Pose &out) {
  if (pose_history_count == 0) return false;
  // Walk from the newest sample back to the first one at or before t_ms.
  int newer = (pose_history_next - 1 + POSE_HISTORY_LEN) % POSE_HISTORY_LEN;
  if ((int32_t)(t_ms - pose_history[newer].ms) >= 0) {
    out = pose_history[newer].pose;  // at or after the newest: best available
    return true;
  }
  for (int k = 1; k < pose_history_count; ++k) {
    const int older = (newer - 1 + POSE_HISTORY_LEN) % POSE_HISTORY_LEN;
    const PoseSample &a = pose_history[older];
    const PoseSample &b = pose_history[newer];
    if ((int32_t)(t_ms - a.ms) >= 0) {
      const float span = (float)(b.ms - a.ms);
      const float f = span > 0.0f ? (float)(t_ms - a.ms) / span : 0.0f;
      out.x = a.pose.x + f * (b.pose.x - a.pose.x);
      out.y = a.pose.y + f * (b.pose.y - a.pose.y);
      out.theta = wrap_angle(a.pose.theta + f * wrap_angle(b.pose.theta - a.pose.theta));
      return true;
    }
    newer = older;
  }
  return false;  // older than the history covers
}

void pose_reset() {
  clear_pose_history();
  current_pose = Pose();
  last_angular_speed = 0.0f;
  have_heading_sample_time = false;
  last_euler_read_ok = false;
  pending_flow_fwd_m = pending_flow_left_m = 0.0f;
  flow_blind = false;
  blind_skipped_x = blind_skipped_y = 0.0f;

  if (imu_ready) {
    last_euler_read_ok = read_heading(last_heading_deg_raw);
    if (last_euler_read_ok) {
      last_heading_sample_us = micros();
      last_heading_sample_ms = millis();
      have_heading_sample_time = true;
      // BNO055 Euler heading is degrees, clockwise-positive, 0-360.
      // Convert to radians, counter-clockwise-positive, and remember it so
      // that the current heading reads as theta = 0 right now.
      theta_offset = wrap_angle(-radians(last_heading_deg_raw));
    }
  }

  if (flow_ready) {
    // Throw away counts accumulated before this point (e.g. during init).
    int16_t dx, dy;
    flow_sensor->readMotionCount(&dx, &dy);
  }
}

void pose_set_position(float x_m, float y_m) {
  clear_pose_history();  // earlier samples are in the old frame
  current_pose.x = x_m;
  current_pose.y = y_m;
  // theta is deliberately left untouched - see pose.h for why.
}

void pose_nudge_position(float dx_m, float dy_m) {
  current_pose.x += dx_m;
  current_pose.y += dy_m;
  for (int i = 0; i < pose_history_count; ++i) {
    pose_history[i].pose.x += dx_m;
    pose_history[i].pose.y += dy_m;
  }
}

void pose_set_stationary_filter(bool enabled) {
  stationary_filter = enabled;
}

void pose_set_flow_blind(bool blind, bool apply_skipped) {
  if (blind == flow_blind) return;
  if (blind) {
    blind_skipped_x = blind_skipped_y = 0.0f;
  } else if (apply_skipped) {
    pose_nudge_position(blind_skipped_x, blind_skipped_y);
  }
  flow_blind = blind;
}

bool pose_flow_blind() {
  return flow_blind;
}

void pose_set_heading(float theta_rad) {
  clear_pose_history();  // earlier samples are in the old frame
  // theta = heading_ccw - theta_offset, so shift the offset by the change.
  theta_offset = wrap_angle(theta_offset + current_pose.theta - theta_rad);
  current_pose.theta = wrap_angle(theta_rad);
}

// ---------------------------------------------------------------------------
// Localisation
// ---------------------------------------------------------------------------

void pose_resume_after_collection() {
  // Translation during the stopped collection interval is unobserved. Preserve
  // x/y, discard accumulated flow, and reacquire absolute heading without
  // applying a rotation correction for an interval whose flow was discarded.
  pending_flow_fwd_m = pending_flow_left_m = 0.0f;
  discard_stationary_flow = true;
}

// One integration step; pose_update() wraps it to record the result.
#if ENABLE_BENCH_TESTS
static FlowDiagnostic flow_diagnostic;

// Same SPI transaction as the Bitcraze library's (private) registerRead().
static uint8_t flow_register_read(uint8_t reg) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  digitalWrite(flow_cs_pin, LOW);
  delayMicroseconds(50);
  SPI.transfer(reg & 0x7F);
  delayMicroseconds(50);
  const uint8_t value = SPI.transfer(0);
  delayMicroseconds(100);
  digitalWrite(flow_cs_pin, HIGH);
  SPI.endTransaction();
  return value;
}

FlowDiagnostic pose_flow_diagnostic() { return flow_diagnostic; }
#endif

static Pose integrate_step() {
#if ENABLE_BENCH_TESTS
  flow_diagnostic = FlowDiagnostic{};
#endif
  float d_theta = 0.0f;                 // heading change this step (rad, CCW+)
  float theta_mid = current_pose.theta; // heading half-way through the step
  last_angular_speed = NAN; // No measurement must not look like zero rotation.
  last_euler_read_ok = false;
  const bool had_heading = have_heading_sample_time;

  // --- Heading from BNO055 ---
  if (imu_ready) {
    last_euler_read_ok = read_heading(last_heading_deg_raw);

    // A failed transaction leaves the last valid heading untouched.
    if (last_euler_read_ok) {
      const uint32_t sample_us = micros();
      float heading_ccw = -radians(last_heading_deg_raw);
      // If reset could not read the IMU, establish the origin on recovery,
      // keeping whatever heading was set (0 after reset, or the start layout's).
      if (!have_heading_sample_time) theta_offset = wrap_angle(heading_ccw - current_pose.theta);
      float new_theta = wrap_angle(heading_ccw - theta_offset);

      d_theta = wrap_angle(new_theta - current_pose.theta);   // wrap-safe across +-pi
      // Time the heading samples, including sensor reads and loop delays.
      // Unsigned subtraction also handles micros() wrapping around.
      const uint32_t elapsed_us = sample_us - last_heading_sample_us;
      if (have_heading_sample_time && elapsed_us > 0) {
        last_angular_speed = d_theta / (elapsed_us * 1.0e-6f);
      }
      last_heading_sample_us = sample_us;
      last_heading_sample_ms = millis();
      have_heading_sample_time = true;
      theta_mid = wrap_angle(current_pose.theta + 0.5f * d_theta);
      current_pose.theta = new_theta;
    }
  }

  // --- Position from PMW3901 optical flow, rotated into the global frame ---
  if (flow_ready) {
    flow_sensor->readMotionCount(&last_dx_counts, &last_dy_counts);
#if ENABLE_BENCH_TESTS
    flow_diagnostic.read = true;  // transaction attempted, not a tracking-quality flag
    flow_diagnostic.raw_x = last_dx_counts;
    flow_diagnostic.raw_y = last_dy_counts;
    // PMW3901 registers: 0x07 SQUAL, 0x0B Shutter_Lower, 0x0C Shutter_Upper.
    flow_diagnostic.squal = flow_register_read(0x07);
    flow_diagnostic.shutter = ((uint16_t)flow_register_read(0x0C) << 8) | flow_register_read(0x0B);
    flow_diagnostic.quality_read = true;
#endif
    if (discard_stationary_flow) {
      pending_flow_fwd_m = pending_flow_left_m = 0.0f;
      discard_stationary_flow = !last_euler_read_ok;
      last_angular_speed = NAN;
      return current_pose;
    }

    // Sensor axes -> body frame (x forward, y left), counts -> metres.
    float fwd_counts  = FLOW_SWAP_XY ? last_dy_counts : last_dx_counts;
    float left_counts = FLOW_SWAP_XY ? last_dx_counts : last_dy_counts;
    float sx = fwd_counts  * FLOW_SIGN_FWD  * FLOW_METERS_PER_COUNT;
    float sy = left_counts * FLOW_SIGN_LEFT * FLOW_METERS_PER_COUNT;
    // Undo the sensor's mounting rotation (rotate clockwise by the mount angle).
    static const float mount_c = cosf(radians(FLOW_MOUNT_ANGLE_DEG));
    static const float mount_s = sinf(radians(FLOW_MOUNT_ANGLE_DEG));
    const float sx_raw = sx;
    sx = sx_raw * mount_c + sy * mount_s;
    sy = -sx_raw * mount_s + sy * mount_c;
#if ENABLE_BENCH_TESTS
    flow_diagnostic.sensor_forward_m = sx;
    flow_diagnostic.sensor_left_m = sy;
#endif

    // Keep flow and heading changes over the same interval across a brief
    // failed read. Do not rotate flow into the map using a stale heading.
    if (!had_heading) {
      pending_flow_fwd_m = pending_flow_left_m = 0.0f;
      return current_pose;
    }
    pending_flow_fwd_m += sx;
    pending_flow_left_m += sy;
    if (!last_euler_read_ok) return current_pose;
    sx = pending_flow_fwd_m;
    sy = pending_flow_left_m;
    pending_flow_fwd_m = pending_flow_left_m = 0.0f;

    // An off-centre sensor sweeps along an arc when the robot rotates and
    // reports that as movement: reported = true + (R(d_theta) - I) * offset.
    // Subtract it so spinning on the spot doesn't move the pose.
    float c = cosf(d_theta), s = sinf(d_theta);
    float corr_x = (c - 1.0f) * FLOW_OFFSET_FWD_M  - s * FLOW_OFFSET_LEFT_M;
    float corr_y = s * FLOW_OFFSET_FWD_M + (c - 1.0f) * FLOW_OFFSET_LEFT_M;
    float dx_body = sx - corr_x;
    float dy_body = sy - corr_y;
#if ENABLE_BENCH_TESTS
    flow_diagnostic.integrated = true;
    flow_diagnostic.correction_forward_m = corr_x;
    flow_diagnostic.correction_left_m = corr_y;
#endif

    // Rotate into the world frame using the mid-step heading, then accumulate.
    float ct = cosf(theta_mid);
    float st = sinf(theta_mid);
    const float wx = dx_body * ct - dy_body * st;
    const float wy = dx_body * st + dy_body * ct;

    if (flow_blind) {
      blind_skipped_x += wx;  // kept in case the blind ends without a position fix
      blind_skipped_y += wy;
    } else if (stationary_filter && motors_stopped_ms() > STOPPED_GRACE_MS) {
      // Motors off and the roll-on is over: the robot can't be moving, so any
      // flow is noise or phantom motion. Ignore it.
    } else {
      current_pose.x += wx;
      current_pose.y += wy;
    }
  }

  return current_pose;
}

Pose pose_current() {
  return current_pose;
}

Pose pose_update() {
  const Pose p = integrate_step();
  // Only samples with a fresh heading are trustworthy for placing readings.
  if (last_euler_read_ok) record_pose_sample(millis(), p);
  return p;
}

float angular_speed_rad_s() {
    return last_angular_speed;
  }

bool pose_imu_ready() {
  return imu_ready;
}

bool pose_flow_ready() {
  return flow_ready;
}

bool pose_heading_valid() {
  return imu_ready && last_euler_read_ok && !pose_heading_stale();
}

bool pose_heading_stale() {
  return !imu_ready || !have_heading_sample_time ||
         (uint32_t)(millis() - last_heading_sample_ms) >= HEADING_TIMEOUT_MS;
}

void pose_get_calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t &mag) {
  sys = gyro = accel = mag = 0;
  if (imu_ready) bno.getCalibration(&sys, &gyro, &accel, &mag);
}
