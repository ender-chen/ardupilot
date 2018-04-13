#pragma once
#include <AP_Math/AP_Math.h>
#define COLLECTING_SAMPLE_LIMIT_VIBRATION 1
#define	CAL_STARTED_WAITING_LIMIT_TIME 30 * 60 * 1000
#define COLLECTING_SAMPLE_LIMIT_TIME 30 * 60 * 1000

enum temperature_cal_status_t
{
	TEMP_CAL_NOT_STARTED = 0,
	TEMP_CAL_STARTED_WAITING = 1,
	TEMP_CAL_COLLECTING_SAMPLE = 2,
	TEMP_CAL_SUCCESS = 3,
	TEMP_CAL_FAILED = 4
};

enum temperature_cal_error_code_t {
	TEMP_CAL_ERROR_COLLECTING_TOO_LONG = 0,
	TEMP_CAL_ERROR_STARTED_WAITING_TOO_LONG = 1,
	TEMP_CAL_ERROR_CAN_NOT_ACCEPT_SAMPLE = 2
};

struct PACKED Temperature_offset {
	Vector3f offset;
	float temperature;
};

class TempCalibrator {
public:
	TempCalibrator();

	void start();

	void new_sample(const Vector3f& delta_velocity, float dt);

	void new_temperature(const float temperature);

	const float get_temperature() const;

	void set_scale_z(const float scale_z);


	void set_error_code(enum error_code);

	enum temperature_cal_status_t get_status() const {
		return _status;
	}

	enum temperature_cal_error_code_t get_error_code() const {
		return _error_code;
	}

	Temperature_offset* get_calibration_result(uint32_t& total_index) const {
		total_index = _total_index;
		return Temperature_offset;
	}

private:
	
	float _temperature;
	float _start_temperature;
	float _end_temperature;
	float _temperature_ratio;
	uint32_t _conf_samples_time;
	uint32_t _total_index;
	
	float _scale_z;

	Temperature_offset* _temperature_offset;

	bool _request_start;

	enum temperature_cal_status_t _status;
	enum temperature_cal_error_code_t _error_code;
};