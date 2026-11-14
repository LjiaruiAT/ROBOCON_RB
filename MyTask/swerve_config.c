#include "swerve_config.h"
#include "config.h"
#include "can.h"

void Swerve_Config(SwerveModule_t *mods)
{
    static const float pos_x[SWERVE_COUNT] = {  0.21175f,  0.21175f, -0.21175f, -0.21175f };
    static const float pos_y[SWERVE_COUNT] = {  0.21175f, -0.21175f, -0.21175f,  0.21175f };

    static const uint32_t steer_id[SWERVE_COUNT] = { 0x201, 0x202, 0x203, 0x204 };
    static const uint32_t drive_id[SWERVE_COUNT] = { 0x01, 0x02, 0x03, 0x04 };

    static GPIO_TypeDef * const key_port[SWERVE_COUNT] = { GPIOC, GPIOC, GPIOB, GPIOB };
    static const uint16_t        key_pin [SWERVE_COUNT] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_0, GPIO_PIN_1 };

    static const float dir_drive_tab[SWERVE_COUNT] = {  -1.0f, -1.0f, -1.0f,  1.0f };
    static const float dir_steer_tab[SWERVE_COUNT] = { -1.0f, -1.0f, -1.0f, -1.0f };
    static const float zero_bias_tab[SWERVE_COUNT] = {  0.0f,  90.0f,  0.0f,  -90.0f };

    if (mods == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < SWERVE_COUNT; i++)
    {
        SwerveModule_t *m = &mods[i];

        Swerve_ModuleInit(m);

        m->steer.ID       = steer_id[i];
        m->steer.hcan     = &hcan1;
        m->drive.hcan     = &hcan2;
        m->drive.motor_id = drive_id[i];

        m->key_port = key_port[i];
        m->key_pin  = key_pin[i];

        m->x  = pos_x[i];
        m->y  = pos_y[i];
        m->steer_gear     = 108.0f;
        m->zero_deg       = 0.0f;
        m->zero_bias_deg  = zero_bias_tab[i];
        m->dir_steer      = dir_steer_tab[i];
        m->dir_drive      = dir_drive_tab[i];
        m->erpm_per_mps   = DRIVE_ERPM_PER_MPS;
        m->steer_deadband = 2.0f;

        m->steer.pos_pid.Kp = 2.0f;
        m->steer.pos_pid.Ki = 0.0f;
        m->steer.pos_pid.Kd = 0.0f;
        m->steer.pos_pid.limit = 6000.0f;
        m->steer.pos_pid.output_limit = 6000.0f;

        m->steer.vel_pid.Kp = 5.0f;
        m->steer.vel_pid.Ki = 0.0f;
        m->steer.vel_pid.Kd = 0.0f;
        m->steer.vel_pid.limit = 5000.0f;
        m->steer.vel_pid.output_limit = 10000.0f;

        m->drive_pid.Kp = 0.063f;
        m->drive_pid.Ki = 0.0f;
        m->drive_pid.Kd = 0.0f;
        m->drive_pid.limit = 2000.0f;
        m->drive_pid.output_limit = 30.0f;
    }
}
