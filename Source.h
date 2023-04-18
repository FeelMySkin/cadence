#ifndef SOURCE_H
#define SOURCE_H

#include "defines.h"



uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; //Handle current connection

ble_sensor_location_t sup_loc[] = {BLE_SENSOR_LOCATION_LEFT_CRANK, BLE_SENSOR_LOCATION_RIGHT_CRANK};

ble_uuid_t m_adv_uuids[] = {
	{BLE_UUID_CYCLING_SPEED_AND_CADENCE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

void advertising_start(bool erase_bond);

#endif