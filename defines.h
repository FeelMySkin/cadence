#ifndef DEFINES_H
#define DEFINES_H
#include "peripherals.h"

#include "app_error.h"
#include "nrf_ble_gatt.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_cscs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#define DEVICE_NAME					"My Cadence"
#define MANUFACTURER_NAME			"FeelMySkin"

#define APP_BLE_OBSERVER_PRIO		3 //BLE Observer priority
#define APP_BLE_CONN_CFG_TAG		1 //SoftDevice config tag.

#define APP_ADV_INTERVAL			40 //25ms (0.625ms step)

#define APP_ADV_DURATION			18000 //180 seconds (10ms step)

#define BATTERY_LEVEL_MEAS_INTERV	APP_TIMER_TICKS(2000)
#define CSCS_INTERVAL				1000 //msec
#define BMI_INTERBAL				80 //msec

#define MIN_CONN_INTERVAL			MSEC_TO_UNITS(500, UNIT_1_25_MS) //Minimum connection interval
#define MAX_CONN_INTERVAL			MSEC_TO_UNITS(1000,UNIT_1_25_MS) //Maximum connection interval
#define SLAVE_LATENCY				0
#define CONN_SUP_TIMEOUT			MSEC_TO_UNITS(4000,UNIT_10_MS) //Connection supervisory, 4sec
#define FIRST_CONN_PARAMS_DELAY		APP_TIMER_TICKS(5000) //Time from initiationg event to first update
#define NEXT_CONN_PARAMS_DELAY		APP_TIMER_TICKS(30000) //Next update event
#define MAX_CONN_PARAMS_COUNT		3//Number of attempts

#define SEC_PARAM_BOND				1 //perform bonding
#define SEC_PARAM_MITM				0 //MITM protection
#define SEC_PARAM_LESC				0 //LE Secure Connection
#define SEC_PARAM_KEYPRESS			0 //Keypress notification
#define SEC_PARAM_IO_CAPABILITIES	BLE_GAP_IO_CAPS_NONE //NO IO capabilities
#define SEC_PARAM_OOB				0 //No Out Of Band data.
#define SEC_PARAM_MIN_KEY_SIZE		7 // Min encription key size
#define SEC_PARAM_MAX_KEY_SIZE		16 // Max encription key size

#define DEAD_BEEF					0xDEADBEEF //Error stamp

#endif
