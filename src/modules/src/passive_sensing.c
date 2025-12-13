/**
 * passive_sensing.c
 *
 * UKF-based passive sensing (external force proxy) for Crazyflie.
 *
 * State:
 *   x = [f_ex, f_ey, f_ez]  (external force in body frame) [N]
 *
 * Process (random walk):
 *   x_k = x_{k-1} + w_k,  w ~ N(0, Q*dt)
 *
 * Measurement:
 *   z = m*a_body + m*g_body - F_thrust_body   (a force residual) [N]
 *   h(x) = x  (we measure force directly via the residual model)
 *
 * Notes:
 * - This follows the paper’s “external force random walk + Kalman filtering”
 *   idea, implemented as a small UKF suitable for firmware.
 * - Torque is not estimated here (left as zeros).
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "log.h"
#include "param.h"
#include "stabilizer_types.h"

#include "passive_sensing.h"

#define PS_NX 3
#define PS_NS (2 * PS_NX + 1)

// -----------------------------------------------------------------------------
// Tunable parameters
// -----------------------------------------------------------------------------

/** Mass [kg] */
static float ps_mass_kg = 0.032f;

/** Thrust coefficient [N / thrust_unit] */
static float ps_k_thrust = 1.0e-6f;

/**
 * Process noise strength for random-walk external force.
 * Units: [N^2 / s]
 * Bigger => force estimate can change faster (less smooth).
 */
static float ps_q_force = 0.10f;

/**
 * Measurement noise variance.
 * Units: [N^2]
 * Bigger => trust the force residual measurement less (more smooth).
 */
static float ps_r_force = 0.50f;

// -----------------------------------------------------------------------------
// Internal state (UKF)
// -----------------------------------------------------------------------------

typedef struct {
  float x[PS_NX];               // state mean
  float P[PS_NX][PS_NX];        // state covariance (symmetric)
} PassiveUKFState;

static PassiveUKFState ukf;
static passiveSensingOutput_t psOut;

// -----------------------------------------------------------------------------
// Small linear algebra helpers (3x3)
// -----------------------------------------------------------------------------

static void mat3Zero(float A[3][3]) {
  memset(A, 0, 9 * sizeof(float));
}

static void mat3Copy(const float A[3][3], float B[3][3]) {
  memcpy(B, A, 9 * sizeof(float));
}

static void mat3AddDiag(float A[3][3], float d0, float d1, float d2) {
  A[0][0] += d0;
  A[1][1] += d1;
  A[2][2] += d2;
}

static void mat3MulVec(const float A[3][3], const float v[3], float out[3]) {
  out[0] = A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2];
  out[1] = A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2];
  out[2] = A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2];
}

static void vec3AddScaled(float a[3], const float b[3], float s) {
  a[0] += s * b[0];
  a[1] += s * b[1];
  a[2] += s * b[2];
}

static void vec3Sub(const float a[3], const float b[3], float out[3]) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

static float vec3Dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Cholesky decomposition for symmetric positive definite 3x3.
// Returns true on success, and writes L such that P = L*L^T (lower triangular).
static bool chol3(const float P[3][3], float L[3][3]) {
  mat3Zero(L);

  // L00
  if (P[0][0] <= 1e-12f) return false;
  L[0][0] = sqrtf(P[0][0]);

  // L10, L11
  L[1][0] = P[1][0] / L[0][0];
  float t11 = P[1][1] - L[1][0] * L[1][0];
  if (t11 <= 1e-12f) return false;
  L[1][1] = sqrtf(t11);

  // L20, L21, L22
  L[2][0] = P[2][0] / L[0][0];
  L[2][1] = (P[2][1] - L[2][0] * L[1][0]) / L[1][1];
  float t22 = P[2][2] - (L[2][0] * L[2][0] + L[2][1] * L[2][1]);
  if (t22 <= 1e-12f) return false;
  L[2][2] = sqrtf(t22);

  return true;
}

// Invert a diagonal 3x3 matrix represented by its diagonal entries.
// Returns false if any diagonal entry is too small.
static bool invDiag3(const float d[3], float invD[3]) {
  for (int i = 0; i < 3; i++) {
    if (fabsf(d[i]) < 1e-12f) return false;
    invD[i] = 1.0f / d[i];
  }
  return true;
}

// -----------------------------------------------------------------------------
// Math helpers for attitude/gravity
// -----------------------------------------------------------------------------

/** Convert quaternion q=[x,y,z,w] to rotation matrix R (body -> world). */
static void quatToRotationMatrix(const float qx, const float qy, const float qz,
                                 const float qw, float R[3][3]) {
  const float xx = qx * qx;
  const float yy = qy * qy;
  const float zz = qz * qz;
  const float xy = qx * qy;
  const float xz = qx * qz;
  const float yz = qy * qz;
  const float wx = qw * qx;
  const float wy = qw * qy;
  const float wz = qw * qz;

  R[0][0] = 1.0f - 2.0f * (yy + zz);
  R[0][1] = 2.0f * (xy - wz);
  R[0][2] = 2.0f * (xz + wy);

  R[1][0] = 2.0f * (xy + wz);
  R[1][1] = 1.0f - 2.0f * (xx + zz);
  R[1][2] = 2.0f * (yz - wx);

  R[2][0] = 2.0f * (xz - wy);
  R[2][1] = 2.0f * (yz + wx);
  R[2][2] = 1.0f - 2.0f * (xx + yy);
}

/** Rotate v_world into body: v_body = R^T v_world */
static void rotateWorldToBody(const float R[3][3], const Axis3f* vWorld, Axis3f* vBody) {
  vBody->x = R[0][0] * vWorld->x + R[1][0] * vWorld->y + R[2][0] * vWorld->z;
  vBody->y = R[0][1] * vWorld->x + R[1][1] * vWorld->y + R[2][1] * vWorld->z;
  vBody->z = R[0][2] * vWorld->x + R[1][2] * vWorld->y + R[2][2] * vWorld->z;
}

static float norm3Axis(const Axis3f* v) {
  return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

// -----------------------------------------------------------------------------
// Measurement construction: force residual z (body frame) [N]
// -----------------------------------------------------------------------------

static void computeForceResidualMeasurement(const sensorData_t* sensorData,
                                            const state_t* state,
                                            const control_t* control,
                                            float z[3]) {
  // accel in m/s^2 (sensorData->acc is in G)
  Axis3f accMs2 = {
    sensorData->acc.x * 9.81f,
    sensorData->acc.y * 9.81f,
    sensorData->acc.z * 9.81f
  };

  // gravity in body frame
  float R[3][3];
  quatToRotationMatrix(state->attitudeQuaternion.x,
                       state->attitudeQuaternion.y,
                       state->attitudeQuaternion.z,
                       state->attitudeQuaternion.w,
                       R);

  Axis3f gWorld = {0.0f, 0.0f, -9.81f};
  Axis3f gBody;
  rotateWorldToBody(R, &gWorld, &gBody);

  // thrust (simple model, body +z)
  Axis3f thrustBody = {0.0f, 0.0f, ps_k_thrust * control->thrust};

  // z = m*a + m*g - thrust
  z[0] = ps_mass_kg * accMs2.x + ps_mass_kg * gBody.x - thrustBody.x;
  z[1] = ps_mass_kg * accMs2.y + ps_mass_kg * gBody.y - thrustBody.y;
  z[2] = ps_mass_kg * accMs2.z + ps_mass_kg * gBody.z - thrustBody.z;
}

// -----------------------------------------------------------------------------
// UKF core
// -----------------------------------------------------------------------------

static void ukfSigmaPoints(const float x[3], const float P[3][3],
                           float Xsig[PS_NS][3],
                           float lambda) {
  // Xsig[0] = x
  Xsig[0][0] = x[0];
  Xsig[0][1] = x[1];
  Xsig[0][2] = x[2];

  // Compute sqrt((n+lambda) P) via Cholesky: P = L L^T
  float L[3][3];
  float Psafe[3][3];
  mat3Copy(P, Psafe);

  // Small jitter on diagonal for robustness
  mat3AddDiag(Psafe, 1e-9f, 1e-9f, 1e-9f);

  if (!chol3(Psafe, L)) {
    // If decomposition fails, fall back to tiny diagonal
    mat3Zero(L);
    L[0][0] = 1e-4f;
    L[1][1] = 1e-4f;
    L[2][2] = 1e-4f;
  }

  const float scale = sqrtf((float)PS_NX + lambda);

  // sigma points: x +/- scale * column_i(L)
  for (int i = 0; i < PS_NX; i++) {
    float col[3] = { L[0][i], L[1][i], L[2][i] };

    Xsig[1 + i][0] = x[0] + scale * col[0];
    Xsig[1 + i][1] = x[1] + scale * col[1];
    Xsig[1 + i][2] = x[2] + scale * col[2];

    Xsig[1 + PS_NX + i][0] = x[0] - scale * col[0];
    Xsig[1 + PS_NX + i][1] = x[1] - scale * col[1];
    Xsig[1 + PS_NX + i][2] = x[2] - scale * col[2];
  }
}

static void ukfPredict(float dt,
                       float x[3], float P[3][3],
                       const float Qdiag[3],
                       float lambda,
                       float alpha, float beta) {
  (void)alpha; (void)beta; // weights use these implicitly via lambda setup

  // Process model: random walk => sigma points unchanged in mean,
  // covariance increases by Q*dt.
  // x stays same; P += Q*dt.
  mat3AddDiag(P, Qdiag[0] * dt, Qdiag[1] * dt, Qdiag[2] * dt);
}

static void ukfUpdateForceMeasurement(const float z[3],
                                      float x[3], float P[3][3],
                                      const float Rdiag[3],
                                      float lambda, float alpha, float beta) {
  // UKF parameters
  const int n = PS_NX;
  const float c = (float)n + lambda;

  const float Wm0 = lambda / c;
  const float Wc0 = Wm0 + (1.0f - alpha * alpha + beta);
  const float Wi = 1.0f / (2.0f * c);

  // Sigma points
  float Xsig[PS_NS][3];
  ukfSigmaPoints(x, P, Xsig, lambda);

  // Measurement function h(x) = x (identity), so Zsig = Xsig
  float z_pred[3] = {0};

  // Mean
  z_pred[0] = Wm0 * Xsig[0][0];
  z_pred[1] = Wm0 * Xsig[0][1];
  z_pred[2] = Wm0 * Xsig[0][2];

  for (int i = 1; i < PS_NS; i++) {
    z_pred[0] += Wi * Xsig[i][0];
    z_pred[1] += Wi * Xsig[i][1];
    z_pred[2] += Wi * Xsig[i][2];
  }

  // Innovation covariance S (we keep only diagonal because measurement is axis-wise)
  float Sdiag[3] = { Rdiag[0], Rdiag[1], Rdiag[2] };

  // Cross covariance Pxz (also axis-wise => diagonal)
  float PxzDiag[3] = {0};

  // Accumulate covariances
  for (int i = 0; i < PS_NS; i++) {
    const float w = (i == 0) ? Wc0 : Wi;

    float dx[3], dz_[3];
    vec3Sub(Xsig[i], x, dx);
    vec3Sub(Xsig[i], z_pred, dz_);

    // Since h is identity, dx and dz_ are the same vector in expectation,
    // but we keep them separate for clarity.
    PxzDiag[0] += w * dx[0] * dz_[0];
    PxzDiag[1] += w * dx[1] * dz_[1];
    PxzDiag[2] += w * dx[2] * dz_[2];

    Sdiag[0] += w * dz_[0] * dz_[0];
    Sdiag[1] += w * dz_[1] * dz_[1];
    Sdiag[2] += w * dz_[2] * dz_[2];
  }

  // Kalman gain K (diagonal)
  float invS[3];
  if (!invDiag3(Sdiag, invS)) {
    return; // can't update safely
  }

  float Kdiag[3] = {
    PxzDiag[0] * invS[0],
    PxzDiag[1] * invS[1],
    PxzDiag[2] * invS[2]
  };

  // Update state: x = x + K*(z - z_pred)
  float y[3] = { z[0] - z_pred[0], z[1] - z_pred[1], z[2] - z_pred[2] };
  x[0] += Kdiag[0] * y[0];
  x[1] += Kdiag[1] * y[1];
  x[2] += Kdiag[2] * y[2];

  // Update covariance: P = P - K*S*K^T (diagonal version)
  P[0][0] -= Kdiag[0] * Sdiag[0] * Kdiag[0];
  P[1][1] -= Kdiag[1] * Sdiag[1] * Kdiag[1];
  P[2][2] -= Kdiag[2] * Sdiag[2] * Kdiag[2];

  // Keep symmetry / non-negative
  if (P[0][0] < 1e-9f) P[0][0] = 1e-9f;
  if (P[1][1] < 1e-9f) P[1][1] = 1e-9f;
  if (P[2][2] < 1e-9f) P[2][2] = 1e-9f;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void passiveSensingInit(void) {
  memset(&ukf, 0, sizeof(ukf));
  memset(&psOut, 0, sizeof(psOut));

  // Reasonable starting covariance (uncertainty in force)
  ukf.P[0][0] = 1.0f;
  ukf.P[1][1] = 1.0f;
  ukf.P[2][2] = 1.0f;
}

void passiveSensingUpdate(const sensorData_t* sensorData,
                          const state_t* state,
                          const control_t* control,
                          passiveSensingOutput_t* out,
                          const float dt) {
  // 1) Build measurement z (force residual)
  float z[3];
  computeForceResidualMeasurement(sensorData, state, control, z);

  // 2) UKF parameters (standard)
  // alpha: spread of sigma points, beta: prior knowledge (2 is good for Gaussian), kappa: secondary scaling
  const float alpha = 0.5f;
  const float beta  = 2.0f;
  const float kappa = 0.0f;
  const float lambda = alpha * alpha * ((float)PS_NX + kappa) - (float)PS_NX;

  // 3) Predict: random walk
  const float Qdiag[3] = { ps_q_force, ps_q_force, ps_q_force };
  ukfPredict(dt, ukf.x, ukf.P, Qdiag, lambda, alpha, beta);

  // 4) Update with measurement
  const float Rdiag[3] = { ps_r_force, ps_r_force, ps_r_force };
  ukfUpdateForceMeasurement(z, ukf.x, ukf.P, Rdiag, lambda, alpha, beta);

  // 5) Fill outputs
  psOut.force.x = ukf.x[0];
  psOut.force.y = ukf.x[1];
  psOut.force.z = ukf.x[2];

  psOut.accMag = norm3Axis(&sensorData->acc);   // still in G
  psOut.gyroEnergy = norm3Axis(&sensorData->gyro); // still in deg/s

  // torque not estimated
  psOut.torque.x = 0.0f;
  psOut.torque.y = 0.0f;
  psOut.torque.z = 0.0f;

  if (out) {
    *out = psOut;
  }
}

// -----------------------------------------------------------------------------
// Parameters and logging
// -----------------------------------------------------------------------------

PARAM_GROUP_START(passive)
PARAM_ADD_CORE(PARAM_FLOAT, mass,    &ps_mass_kg)
PARAM_ADD_CORE(PARAM_FLOAT, kthrust, &ps_k_thrust)
PARAM_ADD_CORE(PARAM_FLOAT, qF,      &ps_q_force)
PARAM_ADD_CORE(PARAM_FLOAT, rF,      &ps_r_force)
PARAM_GROUP_STOP(passive)

LOG_GROUP_START(passive)
LOG_ADD(LOG_FLOAT, fx,     &psOut.force.x)
LOG_ADD(LOG_FLOAT, fy,     &psOut.force.y)
LOG_ADD(LOG_FLOAT, fz,     &psOut.force.z)
LOG_ADD(LOG_FLOAT, accMag, &psOut.accMag)
LOG_ADD(LOG_FLOAT, gyroE,  &psOut.gyroEnergy)
LOG_GROUP_STOP(passive)
