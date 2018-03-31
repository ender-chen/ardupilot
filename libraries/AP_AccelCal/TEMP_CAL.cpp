
#include "TEMP_CAL.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>



TempCalibrator::TempCalibrator() :
	_request_start(false),
	_scale_z(0.0f),
	_temperature_offset(nullptr)
{
	
}

void TempCalibrator::start(float start_temperature, float end_temperature, float temperature_ratio, uint32_t conf_samples_time) {
	_start_temperature = start_temperature;
	_end_temperature = end_temperature;
	_temperature_ratio = temperature_ratio;
	_conf_samples_time = conf_samples_time;

	_request_start = true;

}


void TempCalibrator::new_sample(const Vector3f& delta_velocity, float dt) {
	update_status();
	if (_status != TEMP_CAL_COLLECTING_SAMPLE) {
		return;
	}

	float temperature = get_temperature();
	static float last_temperature = temperature;

	if (temperature - last_temperature >= _temperature_ratio)
	{
		static float sum_delta_velocity = 0, sum_delta_time = 0;
		sum_delta_velocity += delta_velocity;
		sum_delta_time += dt;

		static Vector3f last_delta_velocity = delta_velocity;
		if (abs(delta_velocity.length() - last_delta_velocity.length()) > COLLECTING_SAMPLE_LIMIT_VIBRATION)
		{
			_can_not_accept_sample = true;
			set_error_code(TEMP_CAL_ERROR_CAN_NOT_ACCEPT_SAMPLE);
			
		}
		last_delta_velocity = delta_velocity;

		if (sum_delta_time > _conf_samples_time) {
			Vector3f float sample_offset = sum_delta_velocity/ sum_delta_time;
			calculate_offset(const Vector3f& sample_offset,const float temperature);
			sum_delta_time = 0;
			sum_delta_time = 0;
			last_temperature = temperature;
		}
	}
}

void TempCalibrator::calculate_offset(const Vector3f& offset,const float temperature) {
	static uint16_t temperature_index = 0;
	if (temperature_index >= _temperature_index_limit || temperature >= _end_temperature) {
		_total_index = temperature_index;
		_success_calibrated = true;
		return;
	}

	temperature_offset[temperature_index].offset.x = sample.x;
	temperature_offset[temperature_index].offset.y = sample.y;
	temperature_offset[temperature_index].offset.z = sample.z - GRAVITY_MSS / _scale_z;
	temperature_offset[temperature_index].temperature = temperature;

	temperature_index++;
}



void TempCalibrator::new_temperature(float temperature) {
	_temperature = temperature;
}

const float TempCalibrator::get_temperature() const
{
	return _temperature;
}



void update_status()
{
	uint8_t new_status;
	if (_request_clear)
	{
		_request_clear = false;
		new_status = TEMP_CAL_NOT_STARTED;
	}
	switch (_status)
	case TEMP_CAL_NOT_STARTED:
		if (_request_start) {
			_request_start = false;
			new_status = TEMP_CAL_STARTED_WAITING
			_temp_cal_started_waiting_time = AP_HAL::millis();
		}
		break;
	case TEMP_CAL_STARTED_WAITING:
		float temperature = get_temperature()
		if (temperature > _start_temperature) {
			new_status = TEMP_CAL_COLLECTING_SAMPLE;
			_start_collecting_sample_time = AP_HAL::millis();
		}
		if (AP_HAL::millis() - _temp_cal_started_waiting_time > CAL_STARTED_WAITING_LIMIT_TIME) {
			new_status = TEMP_CAL_FAILED;
			set_error_code(TEMP_CAL_ERROR_STARTED_WAITING_TOO_LONG);
		}
		break;
	case TEMP_CAL_COLLECTING_SAMPLE:
		if (AP_HAL::millis() - _start_collecting_sample_time > COLLECTING_SAMPLE_LIMIT_TIME) {
			new_status = TEMP_CAL_FAILED;
			set_error_code(TEMP_CAL_ERROR_COLLECTING_TOO_LONG);
		}
		if (_can_not_accept_sample) {
			_can_not_accept_sample = false;
			new_status = TEMP_CAL_FAILED;
		}
		if (_success_calibrated)
		{
			new_status == TEMP_CAL_SUCCESS
		}
	case TEMP_CAL_SUCCESS:
		break;
	case TEMP_CAL_FAILED:
		break;
	default:
		break;

	if (new_status != _status)
	{
		if (_status == TEMP_CAL_NOT_STARTED && new_status == TEMP_CAL_STARTED_WAITING)
		{
			_temperature_index_limit = (_start_temperature - _end_temperature) / _collecting_ratio;
			_temperature_offset = (struct Temperature_offset*) calloc(_temperature_index_limit, sizeof(struct Temperature_offset));
		}
		if (_status == TEMP_CAL_COLLECTING_SAMPLE && new_status == TEMP_CAL_SUCCESS) {
			requset_save_parameter();
		}
		if (_new_status == TEMP_CAL_FAILED || _new_status == TEMP_CAL_NOT_STARTED)
		{
			if (_temperature_offset != nullptr) {
				free(_temperature_offset);
			}
		}
		_status = new_status;
	}
}

void TempCalibrator::set_error_code(enum error_code)
{
	_error_code = error_code;	
}

void TempCalibrator::requset_save_parameter()
{
	//use function pointer()
}

void TempCalibrator::clear() {
}


