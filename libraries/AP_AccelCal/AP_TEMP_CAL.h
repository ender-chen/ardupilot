#pragma once

#include <GCS_MAVLINK/GCS_MAVLINK.h>
#include "TEMP_CAL.h"
#include "AP_AccelCal.h"
#include <StorageManager/StorageManager.h>
#include <Dataflash/Dataflash.h>

#define AP_TEMP_MAX_NUM_CLIENTS 4
#define AP_TEMPERATURE_EEPROM_VERSION           0x0001  // version number stored in first four bytes of eeprom.  increment this by one when eeprom format is changed
#define AP_TEMPERATURE_EEPROM_SIZE		        15      // size in bytes of all mission commands


class GCS_MAVLINK;

class GCS_MAVLINK

class AP_TempCal {
public:
	AP_TempCal(Dataflash* dataflash):
	_started(false),
	_dataflash(dataflash),
	{
	}

	void start(GCS_MAVLINK *gcs);

	void cancel();

	void update();

	temperature_cal_status_t get_status() { return _status;}

	temperature_cal_error_code_t get_error_code() { return _error_code; }

	static void register_client(AP_AccelCal_Client *client);

private:
	DataFlash_Class* _dataflash;
	static StorageAccess _storage;
	
	GCS_MAVLINK *_gcs;
	uint8_t _step;
	temperature_cal_status_t _status;
	temperature_cal_error_code_t _error_code;

    static uint8_t _num_clients;
    static AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];


	void success();
	void fail();
	void clear();


	bool _started;
	bool _saving;


	TempCalibrator *get_calibrator(uint8_t i );
	void _printf(const char*, ...);



}