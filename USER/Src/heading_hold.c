	#include "heading_hold.h"
	#include "includes.h"
	#include "imu.h"

	extern IMU_Data_t imu;

	#define HEADING_HOLD_KP                 6.0f
	#define HEADING_HOLD_KI                 0.0f
	#define HEADING_HOLD_KD                 2.5f
	#define HEADING_HOLD_MAX_WZ             600.0f
	#define HEADING_HOLD_INTEGRAL_LIMIT     200.0f
	#define HEADING_HOLD_ERROR_DEADBAND_DEG 1.0f
	#define HEADING_HOLD_OUTPUT_DIR         1.0f
	#define HEADING_HOLD_SAMPLE_PERIOD_MS   10U

	HeadingHold_t heading_hold;

	static pid_t heading_hold_pid;
	static uint32_t heading_hold_last_sample_ms;

	static float heading_hold_normalize_deg(float angle)
	{
		while (angle > 180.0f) {
			angle -= 360.0f;
		}
		while (angle < -180.0f) {
			angle += 360.0f;
		}
		return angle;
	}

	static float heading_hold_get_yaw_deg(void)
	{
		IMU_GetData(&imu);

		return imu.angle_fused_deg.yaw;
	}

	void HeadingHold_Init(void)
	{
		PID_Struct_Init(&heading_hold_pid,
						HEADING_HOLD_KP,
						HEADING_HOLD_KI,
						HEADING_HOLD_KD,
						(int32_t)HEADING_HOLD_MAX_WZ,
						(int32_t)HEADING_HOLD_INTEGRAL_LIMIT,
						INIT);

		heading_hold.target_yaw_deg = heading_hold_get_yaw_deg();
		heading_hold.current_yaw_deg = heading_hold.target_yaw_deg;
		heading_hold.error_deg = 0.0f;
		heading_hold.output_wz = 0.0f;
		heading_hold.enabled = 1;
		heading_hold.initialized = 1;
		heading_hold_last_sample_ms = HAL_GetTick();
	}

	void HeadingHold_Enable(uint8_t enable)
	{
		heading_hold.enabled = (enable != 0);
		if (!heading_hold.enabled) {
			heading_hold.output_wz = 0.0f;
			pid_reset(&heading_hold_pid,
					  HEADING_HOLD_KP,
					  HEADING_HOLD_KI,
					  HEADING_HOLD_KD);
		}
	}

	void HeadingHold_ResetTargetToCurrent(void)
	{
		if (!heading_hold.initialized) {
			HeadingHold_Init();
			return;
		}

		heading_hold.target_yaw_deg = heading_hold_get_yaw_deg();
		heading_hold.current_yaw_deg = heading_hold.target_yaw_deg;
		heading_hold.error_deg = 0.0f;
		heading_hold.output_wz = 0.0f;
		pid_reset(&heading_hold_pid,
				  HEADING_HOLD_KP,
				  HEADING_HOLD_KI,
				  HEADING_HOLD_KD);
	}

	void HeadingHold_Task(void)
	{
		uint32_t now = HAL_GetTick();
		float yaw_deg;

		if ((uint32_t)(now - heading_hold_last_sample_ms) < HEADING_HOLD_SAMPLE_PERIOD_MS) {
			return;
		}

		heading_hold_last_sample_ms = now;
		yaw_deg = heading_hold_get_yaw_deg();
		heading_hold.current_yaw_deg = yaw_deg;
	}

	float HeadingHold_Update(float manual_wz)
	{
		(void)manual_wz;

		if (!heading_hold.initialized) {
			return 0.0f;
		}

		if (!heading_hold.enabled) {
			heading_hold.output_wz = manual_wz;
			return manual_wz;
		}

		heading_hold.current_yaw_deg = heading_hold_get_yaw_deg();
		heading_hold.error_deg = heading_hold_normalize_deg(heading_hold.target_yaw_deg -
															heading_hold.current_yaw_deg);

		if (fabsf(heading_hold.error_deg) < HEADING_HOLD_ERROR_DEADBAND_DEG) {
			heading_hold.output_wz = 0.0f;
			pid_reset(&heading_hold_pid,
					  HEADING_HOLD_KP,
					  HEADING_HOLD_KI,
					  HEADING_HOLD_KD);
			return 0.0f;
		}

		pid_calc(&heading_hold_pid, 0.0f, heading_hold.error_deg);
		heading_hold.output_wz = heading_hold_pid.out * HEADING_HOLD_OUTPUT_DIR;
		return heading_hold.output_wz;
	}

