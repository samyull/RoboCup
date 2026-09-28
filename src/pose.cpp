#include "pose.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Bitcraze_PMW3901.h>

// ---------------------------------------------------------------------------
// Flow sensor calibration / mounting
// ---------------------------------------------------------------------------


// Lens-to-floor height in metres. See pose.h - CALIBRATE FLOW_METERS_PER_COUNT
// with a known-distance push test once this is set.
float MOUNTING_HEIGHT = 0.08F;
float FLOW_METERS_PER_COUNT = MOUNTING_HEIGHT * 0.0021f;   // ~0.168 mm/count at 80 mm

// Map the sensor's raw dx/dy onto the robot body frame (x = forward, y = left).
// Push the robot forward, then to the left, and watch raw_dx / raw_dy in the
// debug print. Set these so pushing forward makes x increase and pushing
// left makes y increase.
static const bool  FLOW_SWAP_XY   = true;  // true if the sensor's dy is the forward axis
static const float FLOW_SIGN_FWD  = -1.0f;   // flip to -1 if forward push gives negative x
static const float FLOW_SIGN_LEFT = -1.0f;   // flip to -1 if left push gives negative y

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

void pose_reset() {
  current_pose = Pose();
  last_angular_speed = 0.0f;
  have_heading_sample_time = false;
  last_euler_read_ok = false;
  pending_flow_fwd_m = pending_flow_left_m = 0.0f;

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
  current_pose.x = x_m;
  current_pose.y = y_m;
  // theta is deliberately left untouched - see pose.h for why.
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

Pose pose_update() {
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
      // If reset could not read the IMU, establish the origin on recovery.
      if (!have_heading_sample_time) theta_offset = wrap_angle(heading_ccw);
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

    // Rotate into the world frame using the mid-step heading, then accumulate.
    float ct = cosf(theta_mid);
    float st = sinf(theta_mid);

    current_pose.x += dx_body * ct - dy_body * st;
    current_pose.y += dx_body * st + dy_body * ct;
  }

  return current_pose;
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

void pose_print_debug() {
  uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;
  if (imu_ready) {
    bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
  }

  Serial.print("[pose] imu_ready="); Serial.print(imu_ready);
  Serial.print(" flow_ready="); Serial.print(flow_ready);
  Serial.print(" euler_ok="); Serial.print(last_euler_read_ok);
  // getSystemStatus() includes delay(200) in this library: startup only.
  Serial.print(" | cal(sys,gyro,accel,mag)=");
  Serial.print(sys_cal); Serial.print(",");
  Serial.print(gyro_cal); Serial.print(",");
  Serial.print(accel_cal); Serial.print(",");
  Serial.print(mag_cal);
  Serial.print(" | raw_heading_deg="); Serial.print(last_heading_deg_raw, 1);
  if (imu_ready) {
    // Raw (non-fused) gyro reading, direct from the sensor - if this moves
    // when you rotate the robot but heading still doesn't, fusion is the
    // problem. If this ALSO never changes, the chip/wiring itself is dead.
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print(" raw_gyro_z="); Serial.print(gyro.z(), 2);
  }
  Serial.print(" | raw_dx="); Serial.print(last_dx_counts);
  Serial.print(" raw_dy="); Serial.print(last_dy_counts);
  Serial.print(" | x="); Serial.print(current_pose.x, 3);
  Serial.print(" y="); Serial.print(current_pose.y, 3);
  Serial.print(" theta_deg="); Serial.println(degrees(current_pose.theta), 1);
}
