#include "ArmKinematics.h"

#include <math.h>
#include <stddef.h>

#define ARM_EPSILON 1e-12
#define ARM_MAX_JOINT_RADPS 30.0
#define ARM_RADPS_TO_RPM (60.0 / (2.0 * M_PI))
#define ARM_DLS_SIN_LINK_DELTA 0.20
#define ARM_DLS_LAMBDA_MM 30.0

static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int solve_linear_3x3(const double a_in[3][3], const double b_in[3],
                            double x[3])
{
    double mat[3][4];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat[i][j] = a_in[i][j];
        }
        mat[i][3] = b_in[i];
    }

    for (int col = 0; col < 3; ++col) {
        int max_row = col;
        double max_value = fabs(mat[col][col]);

        for (int row = col + 1; row < 3; ++row) {
            double value = fabs(mat[row][col]);
            if (value > max_value) {
                max_value = value;
                max_row = row;
            }
        }

        if (max_value < ARM_EPSILON) {
            return 0;
        }

        if (max_row != col) {
            for (int j = col; j < 4; ++j) {
                double tmp = mat[col][j];
                mat[col][j] = mat[max_row][j];
                mat[max_row][j] = tmp;
            }
        }

        double pivot = mat[col][col];
        for (int j = col; j < 4; ++j) {
            mat[col][j] /= pivot;
        }

        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }

            double factor = mat[row][col];
            for (int j = col; j < 4; ++j) {
                mat[row][j] -= factor * mat[col][j];
            }
        }
    }

    for (int i = 0; i < 3; ++i) {
        x[i] = mat[i][3];
    }

    return 1;
}

static void compute_jacobian(double theta1, double theta2, double theta3,
                             double j[3][3])
{
    double s1 = sin(theta1);
    double c1 = cos(theta1);
    double s2 = sin(theta2);
    double c2 = cos(theta2);
    double s3 = sin(theta3);
    double c3 = cos(theta3);

    double r = ARM_LINK_LENGTH_MM * s2 + ARM_LINK_LENGTH_MM * s3;
    double dr_dtheta2 = ARM_LINK_LENGTH_MM * c2;
    double dr_dtheta3 = ARM_LINK_LENGTH_MM * c3;
    double dz_dtheta2 = -ARM_LINK_LENGTH_MM * s2;
    double dz_dtheta3 = -ARM_LINK_LENGTH_MM * s3;

    j[0][0] = -r * s1;
    j[1][0] = r * c1;
    j[2][0] = 0.0;

    j[0][1] = dr_dtheta2 * c1;
    j[1][1] = dr_dtheta2 * s1;
    j[2][1] = dz_dtheta2;

    j[0][2] = dr_dtheta3 * c1;
    j[1][2] = dr_dtheta3 * s1;
    j[2][2] = dz_dtheta3;
}

static int solve_damped_least_squares_3x3(const double j[3][3],
                                           const double b[3],
                                           double omega[3])
{
    double a[3][3];
    double w[3];
    double lambda2 = ARM_DLS_LAMBDA_MM * ARM_DLS_LAMBDA_MM;

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            a[row][col] = 0.0;
            for (int k = 0; k < 3; ++k) {
                a[row][col] += j[row][k] * j[col][k];
            }
            if (row == col) {
                a[row][col] += lambda2;
            }
        }
    }

    if (!solve_linear_3x3(a, b, w)) {
        return 0;
    }

    for (int col = 0; col < 3; ++col) {
        omega[col] = 0.0;
        for (int row = 0; row < 3; ++row) {
            omega[col] += j[row][col] * w[row];
        }
    }

    return 1;
}

static void zero_motor_speeds(JointMotorSpeeds *speeds)
{
    if (speeds == NULL) {
        return;
    }

    speeds->velocity_1.rpm = 0.0f;
    speeds->velocity_1.rpm_abs = 0.0f;
    speeds->velocity_1.dir = 0U;
    speeds->velocity_2.rpm = 0.0f;
    speeds->velocity_2.rpm_abs = 0.0f;
    speeds->velocity_2.dir = 0U;
    speeds->velocity_3.rpm = 0.0f;
    speeds->velocity_3.rpm_abs = 0.0f;
    speeds->velocity_3.dir = 0U;
    speeds->valid = 0;
}

static uint8_t direction_from_signed_value(double value)
{
    return (value >= 0.0) ? 0U : 1U;
}

double arm_motor_rpm_to_joint_radps(float motor_rpm, double gear_ratio)
{
    return (double)motor_rpm / (gear_ratio * ARM_RADPS_TO_RPM);
}

void arm_motor_speed_from_joint_radps(MotorSpeed *speed,
                                      double joint_radps,
                                      double gear_ratio,
                                      uint8_t invert_dir)
{
    double motor_rpm;

    if (speed == NULL) {
        return;
    }

    motor_rpm = joint_radps * gear_ratio * ARM_RADPS_TO_RPM;
    speed->rpm = (float)motor_rpm;
    speed->rpm_abs = (float)fabs(motor_rpm);
    speed->dir = invert_dir ? direction_from_signed_value(-joint_radps)
                            : direction_from_signed_value(joint_radps);
}

int arm_forward_kinematics(double theta1, double theta2, double theta3,
                           point3d_t *position)
{
    double s2;
    double c2;
    double s3;
    double c3;
    double radius;

    if (position == NULL) {
        return 0;
    }

    s2 = sin(theta2);
    c2 = cos(theta2);
    s3 = sin(theta3);
    c3 = cos(theta3);

    radius = ARM_LINK_LENGTH_MM * s2 + ARM_LINK_LENGTH_MM * s3;

    position->x = (float)(radius * cos(theta1));
    position->y = (float)(radius * sin(theta1));
    position->z = (float)(ARM_LINK_HEIGHT_MM +
                          ARM_LINK_LENGTH_MM * c2 +
                          ARM_LINK_LENGTH_MM * c3);

    return 1;
}

uint8_t ik_solve(float x, float y, float z, ik_result_t *result)
{
    if (result == NULL) {
        return 2U;
    }

    result->theta1 = 0.0f;
    result->theta2 = 0.0f;
    result->theta3 = 0.0f;
    result->status = 0U;

    float rho = sqrtf(x * x + y * y);
    float delta_z = z - (float)ARM_LINK_HEIGHT_MM;
    float reach = sqrtf(rho * rho + delta_z * delta_z);
    float max_reach = (float)(ARM_LINK_LENGTH_MM + ARM_LINK_LENGTH_MM);
    float gamma;
    float half_angle;

    if (rho > max_reach ||
        fabsf(delta_z) > max_reach ||
        reach > max_reach) {
        result->status = 1U;
        return 1U;
    }

    result->theta1 = atan2f(y, x);
    gamma = atan2f(rho, delta_z);
    half_angle = acosf((float)clamp_double(
        reach / (2.0f * (float)ARM_LINK_LENGTH_MM),
        -1.0,
        1.0));

    result->theta2 = gamma - half_angle;
    result->theta3 = gamma + half_angle;
    result->status = 0U;

    return 0U;
}

int compute_joint_velocities(double vx, double vy, double vz,
                             double theta1, double theta2, double theta3,
                             double *dtheta1, double *dtheta2,
                             double *dtheta3)
{
    double j[3][3];
    double b[3] = {vx, vy, vz};
    double omega[3];
    double link_delta_sin;

    if (dtheta1 == NULL || dtheta2 == NULL || dtheta3 == NULL) {
        return 0;
    }

    compute_jacobian(theta1, theta2, theta3, j);
    link_delta_sin = sin(theta3 - theta2);
    if (fabs(link_delta_sin) >= ARM_DLS_SIN_LINK_DELTA &&
        solve_linear_3x3(j, b, omega)) {
        *dtheta1 = omega[0];
        *dtheta2 = omega[1];
        *dtheta3 = omega[2];
        return 1;
    }

    if (!solve_damped_least_squares_3x3(j, b, omega)) {
        *dtheta1 = 0.0;
        *dtheta2 = 0.0;
        *dtheta3 = 0.0;
        return 0;
    }

    *dtheta1 = omega[0];
    *dtheta2 = omega[1];
    *dtheta3 = omega[2];

    return 1;
}

void compute_motor_speeds(double vx, double vy, double vz,
                          double theta1, double theta2, double theta3,
                          double gear_ratio_1, double gear_ratio_2,
                          double gear_ratio_3,
                          JointMotorSpeeds *speeds)
{
    double dtheta1;
    double dtheta2;
    double dtheta3;

    if (speeds == NULL) {
        return;
    }

    zero_motor_speeds(speeds);

    if (!compute_joint_velocities(vx, vy, vz, theta1, theta2, theta3,
                                  &dtheta1, &dtheta2, &dtheta3)) {
        return;
    }

    dtheta1 = clamp_double(dtheta1, -ARM_MAX_JOINT_RADPS, ARM_MAX_JOINT_RADPS);
    dtheta2 = clamp_double(dtheta2, -ARM_MAX_JOINT_RADPS, ARM_MAX_JOINT_RADPS);
    dtheta3 = clamp_double(dtheta3, -ARM_MAX_JOINT_RADPS, ARM_MAX_JOINT_RADPS);

    double rpm1 = dtheta1 * gear_ratio_1 * ARM_RADPS_TO_RPM;
    double rpm2 = dtheta2 * gear_ratio_2 * ARM_RADPS_TO_RPM;
    double rpm3 = dtheta3 * gear_ratio_3 * ARM_RADPS_TO_RPM;

    speeds->velocity_1.rpm = (float)rpm1;
    speeds->velocity_1.rpm_abs = (float)fabs(rpm1);
    speeds->velocity_1.dir = direction_from_signed_value(rpm1);
    speeds->velocity_2.rpm = (float)rpm2;
    speeds->velocity_2.rpm_abs = (float)fabs(rpm2);
    /* 0626 原始控制里 2 号关节速度到电机方向需要取反。 */
    speeds->velocity_2.dir = direction_from_signed_value(-dtheta2);
    speeds->velocity_3.rpm = (float)rpm3;
    speeds->velocity_3.rpm_abs = (float)fabs(rpm3);
    speeds->velocity_3.dir = direction_from_signed_value(dtheta3);
    speeds->valid = 1;
}
