#include "AP_TEMP_CAL.h"

const AP_Param::GroupInfo AP_TempCal::var_info[] = {

    // @Param: S_TEMP
    // @DisplayName: start_temperature
    // @Description: Start temperature for accerlation calibration.
    // @Range: -10 100
    // @Increment: 0.1f
    AP_GROUPINFO("S_TEMP",  0, AP_TempCal, _start_temperature, 0.0f),

    // @User: Advanced
    // @DisplayName: end_temperature
    // @Description: ending temperature for accerlation calibration.
    // @Range: 0 100
    // @Increment: 0.1f
    AP_GROUPINFO("E_TEMP",  0, AP_TempCal, _end_temperature, 60.0f),

    // @User: Advanced
    // @DisplayName: TEMP_RATIO
    // @Description:  temperature ratio for accerlation calibration.
    // @Range: 0.1 10
    // @Increment: 0.1f
    // @User: Advanced
    AP_GROUPINFO("TEMP_RATIO",  0, AP_TempCal, _temperature_ratio, 1.0f),

    AP_GROUPEND
};

StorageAccess AP_TempCal::_storage(StorageManager::Storage_Acc_Temperature_Cal);


const extern AP_HAL::HAL& hal;
static bool _start_collect_sample;

AP_TempCal_Client* AP_TempCal::_clients[AP_TEMP_CAL_MAX_NUM_CLIENTS] {};

void AP_TempCal::update()
{
	if (!get_calibrator(0)) {
		// no calibrators
		return;
	}
	uint8_t new_status[_num_active_calibrators];
	if (_started) {
		TempCalibrator *tempcal;
		uint8_t num_active_calibrators = 0;
		for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
			num_active_calibrators++;
		}
		if (num_active_calibrators != _num_active_calibrators) {
			fail();
			return;
		}

		for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
			new_status[i] = cal->get_status();
		
			if (new_status[i] == TEMP_CAL_WAITING_FOR_TEMP) {
				_printf("IMU%d,TEMP BELOW THE TARGET TEMP WATING ......",i);
			}
			else if (new_status[i] == TEMP_CAL_COLLECTING_SAMPLE) {
				_printf("IMU%d,START COLLECTING SAMPLE .....",i);
				_printf("IMU%d,Temperature is %lf",i,cal->get_temperature());
				_printf("IMU%d,completed %.2lf%",i,cal->get_temperature() / (_end_temperature - _start_temperature));
			}
			else if (new_status[i] == TEMP_CAL_FAILED)
			{
				if (cal->get_error_code() == TEMP_CAL_ERROR_COLLECTING_TOO_LONG)
				{
					_printf("IMU%d, error is COLLECTING_TOO_LONG", i);
				} else if (cal->get_error_code() == TEMP_CAL_ERROR_STARTED_WAITING_TOO_LONG) {
					_printf("IMU%d, error is STARTED_WAITING_TOO_LONG", i);
				} else if (cal->get_error_code() == TEMP_CAL_ERROR_CAN_NOT_ACCEPT_SAMPLE) {
					_printf("IMU%d, error is CAN_NOT_ACCEPT_SAMPLE", i);
				}
			}
		}
		bool calibrate_succeed = true;
		bool temp_cal_failed = false;
		for (uint8_t i=0; i< _num_active_calibrators; i++) {
			if (new_status[i] != TEMP_CAL_SUCCESS) {
				calibrate_succeed = false;
			}
			if (new_status[i] == TEMP_CAL_FAILED) {
				temp_cal_failed = true;
			}
		}
		if (calibrate_successed) {
			_started = false;
			_printf("IMU,saving result");
			if (!write_list_temperature_to_storage()) {
				fail();
				return;
			}
            _printf("IMU,Saved result,trying to reboot");
			for(uint8_t i=0; i<_num_clients; i++) {
	           	if(client_active(i)) {
					_clients[i]->_reset_temperature_calibrations();
					_clients[i]->_reboot_temperature_calibrations();
	           	}
            }
		}
		if (temp_cal_failed) {
			_printf("IMU,failed");
			fail();
			return;
		}
	}
	return;
}

void AP_TempCal::start(GCS_MAVLINK *gcs)
{
	if (gcs == nullptr || _started) {
		return;
	}
	_num_active_calibrators = 0;
	TempCalibrator *cal;
	for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
		cal->start(_start_temperature, _end_temperature, _temperature_ratio);
		_num_active_calibrators++;
	}

	_started = true;
	_gcs = gcs;
}


void AP_TempCal::cancel()
{
	clear();
}
void AP_TempCal::fail()
{
	clear();
}
void AP_TempCal::clear()
{
	if (!_started) {
		return;
	}
	for(uint8_t i=0; i<_num_clients; i++) {
       	if(client_active(i)) {
			_clients[i]->_reset_temperature_calibrations();
       	}
    }
	_started = false;
	clear_storage_offset();
}

void AP_TempCal::register_client(AP_TempCal_Client* client) {
	if (client == nullprt || _num_clients >= AP_TEMPCAL_MAX_NUM_CLIENTS) {
		return;
	}

	for (uint8_t i=0; i<_num_clients; i++) {
		if(_clients[i] == client) {
			return;
		}
	}
	_clients[_num_clients] = client;
	_num_clients++;
}

TempCalibrator* AP_TempCal::get_calibrator(uint8_t index) {
	TempCalibrator* ret;
	for (uint8_t i=0; i<_num_clients; i++) {
		for(uint8_t j=0; (ret = _clients[i]->_temp_get_calibrator(j)); j++) {
			if (index == 0) {
				return ret;
			}
			index--;
		}
	}
	return nullprt;
}


bool AP_TempCal::client_active(uint8_t client_num) 
{
	return (bool)_clients[client_num]->_temp_get_calibrator(0);
}

void AP_TempCal::_printf(const char* fmt, ...)
{
    if (!_gcs) {
        return;
    }
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }
    AP_HAL::UARTDriver *uart = _gcs->get_uart();
    /*
     *     to ensure these messages get to the user we need to wait for the
     *     port send buffer to have enough room
     */
    while (uart->txspace() < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
        hal.scheduler->delay(1);
    }

    _gcs->send_text(MAV_SEVERITY_CRITICAL, msg);
}

void AP_TempCal::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_TEMPERATURE_EEPROM_VERSION) {
        if (clear_storage_offset()) {
            _storage.write_uint32(0, AP_TEMPERATURE_EEPROM_VERSION);
        }
    }
}

bool AP_TempCal::read_list_temperature_from_storage() 
{
	uint16_t pos_in_storage = 4;
	_storage_num_cals = _storage.read_byte(4);
	if (_storage_num_cals <= 0 || _storage_num_cals > 4) {
		return false;
	}

	pos_in_storage = 5;
	_storage_cals_index_limit = new uint16_t[_storage_num_cals];
	for (uint8_t i=0; i < _storage_num_cals; i++) {
		pos_in_storage = 5 + i * 2;
		_storage_cals_index_limit[i] = _storage.read_uint16(pos_in_storage);
		if (_storage_cals_index_limit[i] <= 0) {
			return false;
		}
	}

	_temperature_offset_storage = new float* [_storage_num_cals];
	for (uint16_t i=0; i<_storage_num_cals; i++) {
		_temperature_offset_storage[i] = new float[_storage_cals_index_limit[i]];
	}

	for (uint8_t i=0; i<_storage_num_cals; i++) {
		for (uint16_t j=0; j<_storage_cals_index_limit[i]; j++) {
			pos_in_storage += j * 16;
			if (!_storage.read_block(_temperature_offset_storage[i][j], pos_in_storage, 16)) {
				return false;
			}
			_dataflash->Log_Write_Temperature_Offset(i, _temperature_offset_storage[i][j].temperature, _temperature_offset_storage[i][j].offset.x, _temperature_offset_storage[i][j].offset.y, _temperature_offset_storage[i][j].offset.z);
		}
	}
}

void AP_TempCal::clear_storage_offset() {
	if (_storage_num_cals <= 0 || _storage_num_cals > 4) {
		return;
	}
	if (_temperature_offset_storage != nullptr) {
		for (uint8_t i=0; i < _storage_num_cals; i++) {
			free(_temperature_offset_storage[i]);
		}
		free(_temperature_offset_storage);
	}
	if (_storage_cals_index_limit != nullptr) {
		free(_storage_cals_index_limit);
	}
	return;
}


Temperature_offset* AP_TempCal::get_storage_calibrator_result(uint8_t& storage_num_cals, uint8_t* storage_cals_index_limit) {
	if (_temperature_offset_storage == nullptr) {
		return null;
	} 
	storage_num_cals = _storage_num_cals;
	result = storage_cals_index_limit;

	return _temperature_offset_storage;
}


bool AP_TempCal::write_list_temperature_to_storage()
{
	uint16_t pos_in_storage = 4;
	_storage.write_byte(pos_in_storage, _num_active_calibrators);
	pos_in_storage = pos_in_storage + 1;
	_storage_cals_index_limit = new uint16_t[_num_active_calibrators];
	_temperature_offset_storage = new float *[_num_active_calibrators];
	for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
		_temperature_offset_storage = cal->get_calibration_result(_storage_cals_index_limit[i]);
	}

	for (uint8_t i=0; i<_num_active_calibrators; i++) {
		_storage.write_uint16(pos_in_storage, _storage_cals_index_limit[i]);
		pos_in_storage += 2;
	}

	for (uint8_t i=0; i<_num_active_calibrators; i++) {
		for (uint16_t j=0; j<_storage_cals_index_limit[i]; j++) {
			if (!_storage.write_block(pos_in_storage, _storage_cals_index_limit[i][j], 16)) {
				return false;
			}
			pos_in_storage += 16;
		}
	}
}

