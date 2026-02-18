// OrientationCore.h - Gyro calibration, Madgwick filter, and orientation tracking
// Quaternion-based pan/tilt extraction (gimbal-lock free)
#ifndef ORIENTATION_CORE_H
#define ORIENTATION_CORE_H

#include <Arduino.h>
#include "HtConfig.h"

// Automatic gyroscope bias calibration
class GyroCal {
public:
  GyroCal();

  // Update calibration with new gyro reading (in deg/s)
  void apply(float gx, float gy, float gz);

  // Apply calibration offsets to gyro readings
  void get(float &gx, float &gy, float &gz) const;

  // Check if calibration has stabilized
  bool isStable() const;

  // Manual offset management (for persistence)
  void getOffsets(float &ox, float &oy, float &oz) const;
  void setOffsets(float ox, float oy, float oz, bool assumeStable = false);

  // Reset calibration
  void reset();

private:
  float offset[3];
  float emaError[3];
  bool stable;
  uint32_t samples;
};

// Madgwick orientation filter (gyro + accelerometer fusion)
class MadgwickFilter {
public:
  MadgwickFilter();

  // Initialize filter with sample period and gain
  void begin(float samplePeriodSec, float betaGain = 0.1f);

  // Update filter with gyro (rad/s) and accel (normalized g's)
  void updateIMU(float gx, float gy, float gz,
                 float ax, float ay, float az);

  // Reset quaternion to identity (upright)
  void reset();

  // Get current orientation as Euler angles (radians) - kept for debugging
  void getEuler(float &roll, float &pitch, float &yaw) const;

  // Get raw quaternion (w, x, y, z) for direct angle extraction
  void getQuaternion(float &w, float &x, float &y, float &z) const;

private:
  float beta;           // Filter gain
  float samplePeriod;   // Update period in seconds
  float q0, q1, q2, q3; // Quaternion state
};

// Gimbal-lock-free orientation tracking for headtracker
// Extracts pan (horizontal yaw) and tilt (vertical pitch) directly from
// the Madgwick quaternion by tracking a forward direction vector.
// Works regardless of board mounting angle.
class Orientation {
public:
  Orientation();

  // Initialize orientation tracker
  void begin(float samplePeriodSec);

  // Update with new sensor readings (gyro in deg/s, accel in g's)
  void update(float gxDeg, float gyDeg, float gzDeg,
              float ax, float ay, float az);

  // Re-level: capture current orientation as the zero reference
  void reLevel();

  // Get pan and tilt angles relative to last re-level point (radians, unwrapped)
  // pan = first param, tilt = second param, third = 0 (API compat)
  void getAngles(float &pan, float &tilt, float &unused) const;

  // Get display level angles (radians) from gravity in board/screen frame.
  // These are yaw-independent and intended for the on-screen spirit-level UI.
  void getLevelVector(float &x, float &y) const;

  // Get current offset values
  void getOffsets(float &panOff, float &tiltOff, float &unused) const;

private:
  MadgwickFilter filter;

  // Body-frame "forward" vector (perpendicular to gravity at reLevel time)
  // Determined once at the first reLevel by projecting a body axis onto
  // the plane perpendicular to gravity. This stays fixed because it's in
  // body coordinates (moves with the board).
  float fwdBody[3];  // {x, y, z}
  bool fwdBodyValid;

  // Reference world-frame forward direction at reLevel time
  // fwdBody rotated by curQ into world coordinates, projected onto horizontal plane
  float refFwdWorld[3];  // {x, y, z} -- horizontal unit vector

  // Whether reLevel has been called at least once
  bool hasReference;

  // Angle unwrapping state
  float lastRawPan;
  float lastRawTilt;
  float unwrappedPan;
  float unwrappedTilt;

  // Re-level offsets (so getAngles returns 0 right after reLevel)
  float offsetPan;
  float offsetTilt;

  // Display spirit-level state (gravity-derived, yaw-independent)
  float rawLevelX;
  float rawLevelY;
  float levelOffsetX;
  float levelOffsetY;

  // Display level reference basis captured at re-level for robust axis decoupling
  float refGravBody[3];
  float levelAxisXBody[3];
  float levelAxisYBody[3];
  bool levelBasisValid;

  // Low-pass gravity estimate directly from accelerometer for robust UI leveling
  float accelGravBody[3];
  bool accelGravValid;

  // ---- Quaternion math helpers (static, pure functions) ----

  // Rotate vector v by quaternion q: out = q * v * conj(q)
  static void rotateVectorByQuat(const float q[4], const float v[3], float out[3]);

  // Compute conjugate of quaternion
  static void quatConjugate(const float q[4], float out[4]);

  // Multiply two quaternions: out = a * b
  static void quatMultiply(const float a[4], const float b[4], float out[4]);
};

#endif // ORIENTATION_CORE_H
