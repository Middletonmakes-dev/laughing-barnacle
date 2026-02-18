// OrientationCore.cpp - Implementation of orientation tracking classes
// Ported unchanged from XIAO build - pure math, platform-independent
#include "OrientationCore.h"

// ========== GyroCal Implementation ==========

GyroCal::GyroCal() : stable(false), samples(0) {
  for (int i = 0; i < 3; ++i) {
    offset[i] = 0.0f;
    emaError[i] = 0.0f;
  }
}

void GyroCal::apply(float gx, float gy, float gz) {
  float in[3] = { gx, gy, gz };

  // Three-phase alpha for convergence:
  //   Fast (0-2s):   aggressively find bias from scratch or correct stale NVS offsets
  //   Medium (2-4s): refine to sub-degree accuracy (critical: yaw has no accel correction)
  //   Slow (4s+):    fine-track thermal drift during operation
  float alpha;
  if (samples < HtConfig::GYRO_FAST_SAMPLES) {
    alpha = HtConfig::GYRO_INITIAL_ALPHA;
  } else if (samples < HtConfig::GYRO_MEDIUM_SAMPLES) {
    alpha = HtConfig::GYRO_MEDIUM_ALPHA;
  } else {
    alpha = HtConfig::GYRO_NORMAL_ALPHA;
  }

  for (int i = 0; i < 3; ++i) {
    float value = in[i] - offset[i];

    // Only adjust offset when gyro appears stationary
    if (fabs(value) < HtConfig::GYRO_STATIONARY_THRESHOLD) {
      offset[i] += alpha * value;
      emaError[i] = (1.0f - alpha) * emaError[i] + alpha * fabs(value);
    }
    // When already stable and sensor is moving, don't accumulate error
    // (movement is expected, not a calibration problem)
  }

  samples++;

  // Check if calibration has stabilized
  // IMPORTANT: Once stable, NEVER go back to unstable.
  // The gyro offsets are good enough after initial calibration.
  // Movement is expected behavior, not a calibration problem.
  // The user can always double-tap to recenter if drift accumulates.
  if (!stable && samples > HtConfig::GYRO_STABLE_SAMPLES) {
    bool allStable = true;
    for (int i = 0; i < 3; ++i) {
      if (emaError[i] > HtConfig::GYRO_STABLE_ERROR_MAX) {
        allStable = false;
        break;
      }
    }
    stable = allStable;
  }
}

void GyroCal::get(float &gx, float &gy, float &gz) const {
  gx -= offset[0];
  gy -= offset[1];
  gz -= offset[2];
}

bool GyroCal::isStable() const {
  return stable;
}

void GyroCal::getOffsets(float &ox, float &oy, float &oz) const {
  ox = offset[0];
  oy = offset[1];
  oz = offset[2];
}

void GyroCal::setOffsets(float ox, float oy, float oz, bool assumeStable) {
  offset[0] = ox;
  offset[1] = oy;
  offset[2] = oz;
  stable = assumeStable;
  // Always start from 0 samples so the full fast-convergence phase runs.
  // Even with NVS-seeded offsets, the fast alpha phase will quickly verify/correct them.
  // This prevents latching stability on bad stored offsets.
  samples = 0;
  for (int i = 0; i < 3; ++i) {
    emaError[i] = 0.0f;
  }
}

void GyroCal::reset() {
  for (int i = 0; i < 3; ++i) {
    offset[i] = 0.0f;
    emaError[i] = 0.0f;
  }
  stable = false;
  samples = 0;
}

// ========== MadgwickFilter Implementation ==========

MadgwickFilter::MadgwickFilter()
  : beta(0.1f), samplePeriod(0.01f),
    q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {
}

void MadgwickFilter::begin(float samplePeriodSec, float betaGain) {
  samplePeriod = samplePeriodSec;
  beta = betaGain;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

void MadgwickFilter::updateIMU(float gx, float gy, float gz,
                                float ax, float ay, float az) {
  // Normalize accelerometer measurement
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) return; // Avoid division by zero

  ax /= norm;
  ay /= norm;
  az /= norm;

  // Auxiliary variables to avoid repeated calculations
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // Gradient descent algorithm corrective step
  float f1 = 2.0f * (q1q3 - q0q2) - ax;
  float f2 = 2.0f * (q0q1 + q2q3) - ay;
  float f3 = 2.0f * (0.5f - q1q1 - q2q2) - az;

  float J_11or24 = 2.0f * q2;
  float J_12or23 = 2.0f * q3;
  float J_13or22 = 2.0f * q0;
  float J_14or21 = 2.0f * q1;
  float J_32 = 2.0f * J_14or21;
  float J_33 = 2.0f * J_11or24;

  float grad0 = J_14or21 * f2 - J_11or24 * f1;
  float grad1 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  float grad2 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  float grad3 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize gradient
  norm = sqrtf(grad0 * grad0 + grad1 * grad1 + grad2 * grad2 + grad3 * grad3);
  if (norm > 1e-6f) {
    grad0 /= norm;
    grad1 /= norm;
    grad2 /= norm;
    grad3 /= norm;
  }

  // Compute rate of change of quaternion
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * grad0;
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * grad1;
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * grad2;
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * grad3;

  // Integrate to yield quaternion
  q0 += qDot0 * samplePeriod;
  q1 += qDot1 * samplePeriod;
  q2 += qDot2 * samplePeriod;
  q3 += qDot3 * samplePeriod;

  // Normalize quaternion
  norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm > 1e-6f) {
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
  }
}

void MadgwickFilter::reset() {
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

void MadgwickFilter::getEuler(float &roll, float &pitch, float &yaw) const {
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  roll = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabs(sinp) >= 1.0f)
    pitch = copysignf(PI / 2.0f, sinp);
  else
    pitch = asinf(sinp);

  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  yaw = atan2f(siny_cosp, cosy_cosp);
}

void MadgwickFilter::getQuaternion(float &w, float &x, float &y, float &z) const {
  w = q0;
  x = q1;
  y = q2;
  z = q3;
}

// ========== Orientation Implementation ==========
// Quaternion-based pan/tilt extraction, independent of board mounting angle.
//
// How it works:
//   We define a body-frame "forward" vector (fwdBody) that is perpendicular to
//   gravity, computed once at first reLevel. This vector is fixed in body coords
//   and rotates with the board.
//
//   At reLevel(): rotate fwdBody by the current quaternion into world frame,
//   project onto horizontal plane -> this is the reference world-forward direction.
//
//   At update(): rotate fwdBody by the current quaternion into world frame.
//   Pan = horizontal angle between current world-forward and reference world-forward.
//   Tilt = elevation angle of current world-forward above/below horizontal.
//
//   Both extractions use atan2 -> no gimbal lock, no cross-coupling, and works
//   regardless of how the board is mounted (horizontal, vertical, angled).

Orientation::Orientation()
  : fwdBodyValid(false), hasReference(false),
    lastRawPan(0.0f), lastRawTilt(0.0f),
    unwrappedPan(0.0f), unwrappedTilt(0.0f),
    offsetPan(0.0f), offsetTilt(0.0f) {
  fwdBody[0] = 1.0f; fwdBody[1] = 0.0f; fwdBody[2] = 0.0f;
  refFwdWorld[0] = 1.0f; refFwdWorld[1] = 0.0f; refFwdWorld[2] = 0.0f;
}

void Orientation::begin(float samplePeriodSec) {
  filter.begin(samplePeriodSec, HtConfig::MADGWICK_BETA);
  fwdBodyValid = false;
  hasReference = false;
  lastRawPan = 0.0f;
  lastRawTilt = 0.0f;
  unwrappedPan = 0.0f;
  unwrappedTilt = 0.0f;
  offsetPan = 0.0f;
  offsetTilt = 0.0f;
  fwdBody[0] = 1.0f; fwdBody[1] = 0.0f; fwdBody[2] = 0.0f;
  refFwdWorld[0] = 1.0f; refFwdWorld[1] = 0.0f; refFwdWorld[2] = 0.0f;
}

void Orientation::update(float gxDeg, float gyDeg, float gzDeg,
                         float ax, float ay, float az) {
  // Convert gyro from deg/s to rad/s
  float gx = gxDeg * HtConfig::DEG2RAD;
  float gy = gyDeg * HtConfig::DEG2RAD;
  float gz = gzDeg * HtConfig::DEG2RAD;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  if (!hasReference) return;

  // Get current quaternion
  float curQ[4];
  filter.getQuaternion(curQ[0], curQ[1], curQ[2], curQ[3]);

  // Rotate fwdBody by current quaternion to get current forward in world frame
  float curFwd[3];
  rotateVectorByQuat(curQ, fwdBody, curFwd);

  // --- Extract tilt: elevation angle above/below horizontal ---
  // Tilt is the angle between curFwd and the horizontal plane
  float horizLen = sqrtf(curFwd[0] * curFwd[0] + curFwd[1] * curFwd[1]);
  float rawTilt = atan2f(curFwd[2], horizLen);

  // --- Extract pan: horizontal angle relative to reference direction ---
  // Project current forward onto horizontal plane (drop Z component)
  // Compute signed angle from refFwdWorld to curFwd horizontal projection
  // Using atan2(cross, dot) for signed angle
  float curHorizX = curFwd[0];
  float curHorizY = curFwd[1];
  // Normalize the horizontal projection
  if (horizLen > 1e-6f) {
    curHorizX /= horizLen;
    curHorizY /= horizLen;
  }
  // Signed angle: dot = cos(angle), cross_z = sin(angle)
  float dotH = refFwdWorld[0] * curHorizX + refFwdWorld[1] * curHorizY;
  float crossZ = refFwdWorld[0] * curHorizY - refFwdWorld[1] * curHorizX;
  float rawPan = atan2f(crossZ, dotH);

  // Unwrap pan to prevent servo-destroying jumps at +/-PI boundary
  float panDiff = rawPan - lastRawPan;
  if (panDiff > PI) {
    unwrappedPan -= 2.0f * PI;
  } else if (panDiff < -PI) {
    unwrappedPan += 2.0f * PI;
  }
  unwrappedPan += panDiff;

  // Unwrap tilt (less likely to wrap but handle it for safety)
  float tiltDiff = rawTilt - lastRawTilt;
  if (tiltDiff > PI) {
    unwrappedTilt -= 2.0f * PI;
  } else if (tiltDiff < -PI) {
    unwrappedTilt += 2.0f * PI;
  }
  unwrappedTilt += tiltDiff;

  lastRawPan = rawPan;
  lastRawTilt = rawTilt;
}

void Orientation::reLevel() {
  // Get the current quaternion from the filter
  float curQ[4];
  filter.getQuaternion(curQ[0], curQ[1], curQ[2], curQ[3]);

  // --- Compute fwdBody once (first reLevel only) ---
  // fwdBody is a body-frame vector perpendicular to gravity.
  // It represents "the direction the user looks" in the board's coordinate system.
  // Only needs to be computed once since the board doesn't change shape.
  if (!fwdBodyValid) {
    // Find where gravity points in body frame
    // World gravity = [0, 0, -1], rotate into body frame: gravBody = conj(q) * grav * q
    float gravWorld[3] = {0.0f, 0.0f, -1.0f};
    float qConj[4];
    quatConjugate(curQ, qConj);
    float gravBody[3];
    rotateVectorByQuat(qConj, gravWorld, gravBody);

    // Pick body X-axis as candidate, project onto plane perpendicular to gravity
    float candidate[3] = {1.0f, 0.0f, 0.0f};
    float dot = candidate[0]*gravBody[0] + candidate[1]*gravBody[1] + candidate[2]*gravBody[2];
    if (fabs(dot) > 0.9f) {
      // Body X-axis is nearly parallel to gravity -> use body Y-axis instead
      candidate[0] = 0.0f; candidate[1] = 1.0f; candidate[2] = 0.0f;
      dot = candidate[0]*gravBody[0] + candidate[1]*gravBody[1] + candidate[2]*gravBody[2];
    }

    // Project: subtract gravity component
    fwdBody[0] = candidate[0] - dot * gravBody[0];
    fwdBody[1] = candidate[1] - dot * gravBody[1];
    fwdBody[2] = candidate[2] - dot * gravBody[2];

    // Normalize
    float norm = sqrtf(fwdBody[0]*fwdBody[0] + fwdBody[1]*fwdBody[1] + fwdBody[2]*fwdBody[2]);
    if (norm > 1e-6f) {
      fwdBody[0] /= norm;
      fwdBody[1] /= norm;
      fwdBody[2] /= norm;
    }
    fwdBodyValid = true;
  }

  // --- Compute reference world-forward direction ---
  // Rotate fwdBody by current quaternion into world frame
  float fwdWorld[3];
  rotateVectorByQuat(curQ, fwdBody, fwdWorld);

  // Project onto horizontal plane (zero out Z) and normalize
  refFwdWorld[0] = fwdWorld[0];
  refFwdWorld[1] = fwdWorld[1];
  refFwdWorld[2] = 0.0f;  // horizontal only
  float norm = sqrtf(refFwdWorld[0]*refFwdWorld[0] + refFwdWorld[1]*refFwdWorld[1]);
  if (norm > 1e-6f) {
    refFwdWorld[0] /= norm;
    refFwdWorld[1] /= norm;
  }

  hasReference = true;

  // Reset unwrapped angles so getAngles() returns 0 immediately after reLevel.
  // At this instant, curFwd projected horizontal == refFwdWorld, so rawPan = 0.
  // And tilt relative to horizontal is whatever the current elevation is,
  // but we want tilt=0 at reLevel, so we capture it as an offset.

  // Compute current tilt to use as offset
  float curFwd[3];
  rotateVectorByQuat(curQ, fwdBody, curFwd);
  float horizLen = sqrtf(curFwd[0]*curFwd[0] + curFwd[1]*curFwd[1]);
  float currentTilt = atan2f(curFwd[2], horizLen);

  lastRawPan = 0.0f;
  lastRawTilt = currentTilt;
  unwrappedPan = 0.0f;
  unwrappedTilt = currentTilt;
  offsetPan = 0.0f;
  offsetTilt = currentTilt;  // so getAngles returns tilt=0 right after reLevel
}

void Orientation::getAngles(float &pan, float &tilt, float &unused) const {
  pan = unwrappedPan - offsetPan;
  tilt = unwrappedTilt - offsetTilt;
  unused = 0.0f;
}

void Orientation::getOffsets(float &panOff, float &tiltOff, float &unused) const {
  panOff = offsetPan;
  tiltOff = offsetTilt;
  unused = 0.0f;
}

// ========== Quaternion Math Helpers ==========

void Orientation::rotateVectorByQuat(const float q[4], const float v[3], float out[3]) {
  // Rotate vector v by quaternion q: out = q * v * conj(q)
  // Using the expanded form for efficiency (avoids two full quat multiplies)
  float w = q[0], x = q[1], y = q[2], z = q[3];

  // t = 2 * cross(q.xyz, v)
  float tx = 2.0f * (y * v[2] - z * v[1]);
  float ty = 2.0f * (z * v[0] - x * v[2]);
  float tz = 2.0f * (x * v[1] - y * v[0]);

  // out = v + w * t + cross(q.xyz, t)
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

void Orientation::quatConjugate(const float q[4], float out[4]) {
  out[0] =  q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

void Orientation::quatMultiply(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
