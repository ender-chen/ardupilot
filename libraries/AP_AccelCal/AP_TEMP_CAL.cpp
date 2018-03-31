#include "AP_TEMP_CAL.h"

const AP_Param::GroupInfo AP_Mission::var_info[] = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, ACC_TEMP, _list_total, 0),


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

	if (_started) {
		uint8_t new_status;
		for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
			if (cal->get_status() == TEMP_CAL_FAILED) {
				new_status = TEMP_CAL_FAILED;
				break;
			}
			if (cal->get_status() == TEMP_CAL_NOT_STARTED) {
				new_status = TEMP_CAL_NOT_STARTED;
				break;
			}
			if (cal->get_status() == TEMP_CAL_COLLECTING_SAMPLE) {
				new_status = TEMP_CAL_COLLECTING_SAMPLE;
				break;
			}
			if (cal->get_status() == TEMP_CAL_SUCCESS) {
				new_status = TEMP_CAL_SUCCESS;
				break;
			}
		}






		TempCalibrator *tempcal;
		uint8_t num_active_calibrators = 0;
		for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
			num_active_calibrators++;
		}
		if (num_active_calibrators != _num_active_calibrators) {
			fail();
			return;
		}
		if (_status == TEMP_CAL_WAITING_FOR_TEMP) {
			_printf("TEMP BELOW THE TARGET TEMP WATING ......");
		}
		else if (_status == TEMP_CAL_COLLECTING_SAMPLE) {
			TempCalibrator *cal;
			cal = get_calibrator(0);
			_printf("START COLLECTING SAMPLE .....");
			_printf("Temperature is %lf", cal->get_temperature());
			_printf("completed %.2lf%", cal->get_temperature() / (_end_temperature - _start_temperature));
		}
		else if (new_status == TEMP_CAL_SUCCESS && _status == TEMP_CAL_COLLECTING_SAMPLE) {
			_printf("saving result");
			for(uint8_t i=0; i<_num_clients; i++) {
	            if(client_active(i)) {
					_clients[i]->_temperature_save_calibrations();
	            }
            }
		}
		else if (new_status == TEMP_CAL_FAILED)
		{
			for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
				if (cal->get_error_code() == TEMP_CAL_ERROR_COLLECTING_TOO_LONG)
				{
					_printf("cal%d, error is COLLECTING_TOO_LONG", i);
				} else if (cal->get_error_code() == TEMP_CAL_ERROR_STARTED_WAITING_TOO_LONG) {
					_printf("cal%d, error is STARTED_WAITING_TOO_LONG", i);
				} else if (cal->get_error_code() == TEMP_CAL_ERROR_CAN_NOT_ACCEPT_SAMPLE) {
					_printf("cal%d, error is CAN_NOT_ACCEPT_SAMPLE", i);
				}
			}
		}

		_status = new_status;

	}
}

void TEMP_Cal::start(GCS_MAVLINK *gcs)
{
	if (gcs == nullptr || _started) {
		return;
	}
	_num_active_calibrators = 0;
	TempCalibrator *cal;
	for (uint8_t i=0; (cal = get_calibrator(i)); i++) {
		cal->clear();
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
	if (_started) {
		return;
	}
	for (uint8_t i=0; (cal=get_calibrator(i)); i++) {
		cal->clear();
	}

	_gcs = nullprt;

	_started = false;
	_saving = false;

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


uint16_t AP_TempCal::num_list_max(void) const
{
	return (_storage.size() - 4) / AP_TEMPERATURE_EEPROM_SIZE;
}

void AP_TempCal::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_TEMPERATURE_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_TEMPERATURE_EEPROM_VERSION);
        }
    }
}

bool AP_TempCal::clear()
{
	_list_total.set_and_save(0);
	return true;
}
struct PACKED Temperature_offset_storage
{
	uint8_t index_active_calibrators;
	union PACKED 
	Temperature_offset_Content temperature_offset_content;
};

Temperature_offset_storage *temperature_offset_storage;

union PACKED Temperature_offset_Content {
	struct Temperature_offset temperpature_offset;
	uint8_t bytes[16];
}

bool AP_TempCal::read_list_temperature_from_storage()
{
	for (uint16_t index = 0; index < _total_index; index++)
	{
		uint16_t pos_in_storage = 4 + (index * AP_TEMPERATURE_EEPROM_SIZE);
		temperature_offset_storage[index].index_active_calibrators = _storage.read_byte(pos_in_storage);
		_storage.read_block(temperature_offset_storage[index].temperature_offset_content, pos_in_storage+1, 16);
	}
}

bool AP_TempCal::write_list_temperature_to_storage(uint16_t index, Temperatrue_offset& offset)
{
	for (uint8_t index_active_calibrators = 0; index_active_calibrators < _num_active_calibrators; index_active_calibrators++)
	{
		for (uint16_t index = 0; index < _total_index; index++)
		{
			uint16_t pos_in_storage = 4 + (index * AP_TEMPERATURE_EEPROM_SIZE);
			_storage.write_byte(pos_in_storage, index_active_calibrators);
			_storage.write_block(pos_in_storage+1, Temperatrue_offset[index], 16);
			save_log(index_active_calibrators, Temperatrue_offset[index], index);
			_dataflash->Log_Write_Temperature_Offset(index_active_calibrators, Temperature_offset[index].temperature, Temperature_offset[index].offset.x, Temperature_offset[index].offset.y, Temperature_offset[index].offset.z);
		}
	}
	return true;
}