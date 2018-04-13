#pragma once

#include <GCS_MAVLINK/GCS_MAVLINK.h>
#include "TEMP_CAL.h"
#include "AP_AccelCal.h"
#include <StorageManager/StorageManager.h>
#include <Dataflash/Dataflash.h>
#include <AP_Param/AP_Param.h>

#define AP_TEMP_MAX_NUM_CLIENTS 4
#define AP_TEMPERATURE_EEPROM_VERSION           0x0001  // version number stored in first four bytes of eeprom.  increment this by one when eeprom format is changed
#define AP_TEMPERATURE_EEPROM_SIZE		        16      // size in bytes of all mission commands


class GCS_MAVLINK;

class GCS_MAVLINK

class AP_TempCal {
public:
	AP_TempCal(Dataflash* dataflash):
	_started(false),
	_dataflash(dataflash),
	{
		AP_Param::setup_object_defaults(this, var_info);
		check_eeprom_version();
		if(!read_list_temperature_from_storage()) {
			clear_storage_offset();
		}
	}

	void start(GCS_MAVLINK *gcs);

	void cancel();

	void update();

	temperature_cal_status_t get_status() { return _status;}

	temperature_cal_error_code_t get_error_code() { return _error_code; }

	Temperature_offset** require_offset() { return _temperature_offset_storage; }

	uint16_t *require_limit(uint16_t &storage_num_cals) {
		storage_num_cals = _storage_num_cals;
		return _storage_cals_index_limit;
	}



	static void register_client(AP_AccelCal_Client *client);

	static const struct AP_Param::GroupInfo var_info[];


private:
	AP_Float _start_temperature;
	AP_Float _end_temperature;
	AP_Float _temperature_ratio;

	DataFlash_Class* _dataflash;
	static StorageAccess _storage;
	
	GCS_MAVLINK *_gcs;
	uint8_t _step;
	temperature_cal_status_t _status;
	temperature_cal_error_code_t _error_code;
	uint8_t _storage_num_cals;
	uint8_t _num_active_calibrators;

    static uint8_t _num_clients;
    static AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];


	void success();
	void fail();
	void clear();


	bool _started;
	bool _saving;


	TempCalibrator *get_calibrator(uint8_t i );
	void _printf(const char*, ...);

	Temperature_offset** _temperature_offset_storage;
	uint16_t* _storage_cals_index_limit;


}