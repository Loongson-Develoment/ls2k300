#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM_LINK_LENGTH_MM 120.0
#define ARM_LINK_HEIGHT_MM 0.0

typedef struct {
    float x;
    float y;
    float z;
} point3d_t;

typedef struct {
    float theta1;
    float theta2;
    float theta3;
    uint8_t status;
} ik_result_t;

typedef struct {
    float rpm;
    uint8_t dir;
    float rpm_abs;
} MotorSpeed;

typedef struct {
    MotorSpeed velocity_1;
    MotorSpeed velocity_2;
    MotorSpeed velocity_3;
    int valid;
} JointMotorSpeeds;

/*
 * 根据三个关节实际角度计算末端空间坐标。
 * theta1/theta2/theta3 单位为 rad，输出坐标单位为 mm。
 * theta2/theta3 使用 robotArm 约定：两根杆均为从竖直向上 +Z
 * 开始计量的绝对角，0deg 为竖直向上，90deg 为水平向外，
 * 180deg 为竖直向下。z=0 位于第二轴/下臂旋转轴心；
 * 如需地面坐标，在上层显示或限位逻辑中另加基座高度。
 */
int arm_forward_kinematics(double theta1, double theta2, double theta3,
                           point3d_t *position);

uint8_t ik_solve(float x, float y, float z, ik_result_t *result);

int compute_joint_velocities(double vx, double vy, double vz,
                             double theta1, double theta2, double theta3,
                             double *dtheta1, double *dtheta2,
                             double *dtheta3);

void compute_motor_speeds(double vx, double vy, double vz,
                          double theta1, double theta2, double theta3,
                          double gear_ratio_1, double gear_ratio_2,
                          double gear_ratio_3,
                          JointMotorSpeeds *speeds);

double arm_motor_rpm_to_joint_radps(float motor_rpm, double gear_ratio);

void arm_motor_speed_from_joint_radps(MotorSpeed *speed,
                                      double joint_radps,
                                      double gear_ratio,
                                      uint8_t invert_dir);

#ifdef __cplusplus
}
#endif

#endif
