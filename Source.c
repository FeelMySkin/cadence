#include "Source.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_twi.h"
#include "nrf.h"
#include "math.h"

BLE_BAS_DEF(m_bas); //battery service instance
BLE_CSCS_DEF(m_cscs); //Cycling speed and cadense service instance
NRF_BLE_GATT_DEF(m_gatt); //GATT module instance
NRF_BLE_QWR_DEF(m_qwr); //Queued Write Module context
BLE_ADVERTISING_DEF(m_adv); //Advertising module instance
APP_TIMER_DEF(m_battery_tim); //Battery timer
APP_TIMER_DEF(m_bmi_tim);
APP_TIMER_DEF(m_cscs_tim); //CSC measure timer

static ble_sensor_location_t supported_locations[] =                                /**< Supported location for the sensor location. */
{
    BLE_SENSOR_LOCATION_FRONT_WHEEL,
    BLE_SENSOR_LOCATION_LEFT_CRANK,
    BLE_SENSOR_LOCATION_RIGHT_CRANK,
    BLE_SENSOR_LOCATION_LEFT_PEDAL,
    BLE_SENSOR_LOCATION_RIGHT_PEDAL,
    BLE_SENSOR_LOCATION_FRONT_HUB,
    BLE_SENSOR_LOCATION_REAR_DROPOUT,
    BLE_SENSOR_LOCATION_CHAINSTAY,
    BLE_SENSOR_LOCATION_REAR_WHEEL,
    BLE_SENSOR_LOCATION_REAR_HUB
};


bool sending = false;
bool sent = false;
bool receiving = false;
bool received = false;
uint8_t recv_byte;

static uint32_t m_cumulative_wheel_revs;                                            /**< Cumulative wheel revolutions. */
uint32_t test_cadence = 0, wheel_tst = 0;
uint32_t last_evt_time = 0;


/*------Handlers------**/
static void idle_state_handle(void)
{
        nrf_pwr_mgmt_run();
}
//

//TODO: update battery
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	
	
    //battery_level_update();
}
//

//TODO: update cscs
static void csc_meas_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    ble_cscs_meas_t cscs_measurement;
	last_evt_time+=1000;
	
	static uint16_t crank_rev_deg = 0;
    static uint16_t cumulative_crank_revs = 0;
    static uint16_t event_time            = 0;

    UNUSED_PARAMETER(p_context);
	struct MMU_Data gyro = BMI_GetGyro();
	double test_summ = fabs(gyro.x) + fabs(gyro.y) + fabs(gyro.z);
	test_cadence = (uint16_t)(test_summ);
	
	crank_rev_deg += test_cadence;
	cumulative_crank_revs += crank_rev_deg/360;
	crank_rev_deg %= 360;
	cscs_measurement.cumulative_crank_revs = cumulative_crank_revs;
	cscs_measurement.is_wheel_rev_data_present = false;
	cscs_measurement.is_crank_rev_data_present = true;
	
    // Per specification event time is in 1/1024th's of a second.
    cscs_measurement.last_crank_event_time =
        event_time + (1024 * (test_cadence - crank_rev_deg) / test_cadence);

   // csc_sim_measurement(&cscs_measurement);

    err_code = ble_cscs_measurement_send(&m_cscs, &cscs_measurement);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
    /*if (m_auto_calibration_in_progress)
    {
        err_code = ble_sc_ctrlpt_rsp_send(&(m_cscs.ctrl_pt), BLE_SCPT_SUCCESS);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
        if (err_code != NRF_ERROR_RESOURCES)
        {
            m_auto_calibration_in_progress = false;
        }
    }*/
}
//

//TODO: check if works
static void m_bmi_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	BMI_ReadData();
	
	
	
	
}
//

//TODO: understand it!
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_INFO("Connected");
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
//

//TODO: understand it!
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //NRF_LOG_INFO("Fast advertising");
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}
//

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
//

//TODO: understand it!
ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t     * p_sc_ctrlpt, ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
		//TODO: wheels_rev set?
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
            m_cumulative_wheel_revs = p_evt->params.cumulative_value;
            break;

        /*case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
            m_auto_calibration_in_progress = true;
            break;*/

        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
}
//

//TODO: understand!
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}
//

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
//


static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}
//

/*------Handlers End------*/


static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_tim,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_cscs_tim,
                                APP_TIMER_MODE_REPEATED,
                                csc_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	
	
	err_code = app_timer_create(&m_bmi_tim,APP_TIMER_MODE_REPEATED,m_bmi_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
//

static void InitBMI()
{
	struct BMI_InitStruct bmi;
	bmi.scl_pin = I2C0_SCL_PIN;
	bmi.sda_pin = I2C0_SDA_PIN;
	bmi.i2c_instance = 0;

	BMI_Init(&bmi);
	while(!BMI_Setup()) ;
	//BMI_SetupIRQ();
}
//

static void timers_start()
{
	ret_code_t err_code;
    uint32_t csc_meas_timer_ticks;

    // Start application timers.
    err_code = app_timer_start(m_battery_tim, BATTERY_LEVEL_MEAS_INTERV, NULL);
    APP_ERROR_CHECK(err_code);

    csc_meas_timer_ticks = APP_TIMER_TICKS(CSCS_INTERVAL);

    err_code = app_timer_start(m_cscs_tim, csc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
	
	uint32_t bmi_timer = APP_TIMER_TICKS(BMI_INTERBAL);
    err_code = app_timer_start(m_bmi_tim, bmi_timer, NULL);
    APP_ERROR_CHECK(err_code);
	
}
//

static void power_mgmt_init()
{
	ret_code_t err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
}
//

static void ble_stack_init()
{
	ret_code_t err_code;
	
	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);
	
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);
	
	NRF_SDH_BLE_OBSERVER(m_ble_observer,APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
//

static void gap_params_init()
{
	ret_code_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t*) DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);
	
	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR);
	APP_ERROR_CHECK(err_code);
	
	memset(&gap_conn_params, 0, sizeof(gap_conn_params));
	
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	
	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}
//

static void gatt_init()
{
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
	APP_ERROR_CHECK(err_code);
}
//

static void sleep_mode_enter(void)
{
	ret_code_t err_code;
    //ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void advertising_init()
{
	ret_code_t err_code;
	ble_advertising_init_t init;
	memset(&init, 0, sizeof(init));
	
	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = true;
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids)/sizeof(m_adv_uuids[0]);
	init.advdata.uuids_complete.p_uuids = m_adv_uuids;
	
	init.config.ble_adv_fast_enabled = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
	
	init.evt_handler = on_adv_evt;
	
	err_code = ble_advertising_init(&m_adv, &init);
	APP_ERROR_CHECK(err_code);
	
	ble_advertising_conn_cfg_tag_set(&m_adv, APP_BLE_CONN_CFG_TAG);
}
//

static void services_init()
{
	ret_code_t err_code;
	ble_cscs_init_t cscs_init;
	ble_bas_init_t bas_init;
	ble_dis_init_t dis_init;
	ble_sensor_location_t sensor_loc;
	nrf_ble_qwr_init_t qwr_init = {0};
	
	qwr_init.error_handler = nrf_qwr_error_handler;
	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);
	
	memset(&cscs_init, 0, sizeof(cscs_init));
	cscs_init.evt_handler = NULL;
	cscs_init.feature = BLE_CSCS_FEATURE_CRANK_REV_BIT | BLE_CSCS_FEATURE_MULTIPLE_SENSORS_BIT;
	
	cscs_init.csc_meas_cccd_wr_sec = SEC_OPEN;
	cscs_init.csc_feature_rd_sec = SEC_OPEN;
	cscs_init.csc_location_rd_sec = SEC_OPEN;
	cscs_init.sc_ctrlpt_cccd_wr_sec = SEC_OPEN;
	cscs_init.sc_ctrlpt_wr_sec = SEC_OPEN;
	
	cscs_init.ctrplt_supported_functions = 	BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED | 
											BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED |
											BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
	cscs_init.ctrlpt_evt_handler = sc_ctrlpt_event_handler;
	cscs_init.list_supported_locations = supported_locations;
    cscs_init.size_list_supported_locations = sizeof(supported_locations) /
                                              sizeof(ble_sensor_location_t);
	
	sensor_loc = BLE_SENSOR_LOCATION_LEFT_CRANK;
	cscs_init.sensor_location = &sensor_loc;
	
	err_code = ble_cscs_init(&m_cscs, &cscs_init);
	APP_ERROR_CHECK(err_code);
	
	memset(&bas_init, 0, sizeof(bas_init));
	
	bas_init.bl_rd_sec = SEC_OPEN;
	bas_init.bl_cccd_wr_sec = SEC_OPEN;
	bas_init.bl_report_rd_sec = SEC_OPEN;
	
	bas_init.evt_handler = NULL;
	bas_init.support_notification = true;
	bas_init.p_report_ref = NULL;
	bas_init.initial_batt_level = 100;
	
	err_code = ble_bas_init(&m_bas, &bas_init);
	APP_ERROR_CHECK(err_code);
	
	memset(&dis_init, 0, sizeof(dis_init));
	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
	dis_init.dis_char_rd_sec = SEC_OPEN;
	
	err_code = ble_dis_init(&dis_init);
	APP_ERROR_CHECK(err_code);
}	
//

static void conn_params_init()
{
	ret_code_t err_code;
	ble_conn_params_init_t conn_params;
	memset(&conn_params, 0, sizeof(conn_params));

	conn_params.p_conn_params = NULL;
	conn_params.first_conn_params_update_delay = FIRST_CONN_PARAMS_DELAY;
	conn_params.next_conn_params_update_delay = NEXT_CONN_PARAMS_DELAY;
	conn_params.max_conn_params_update_count = MAX_CONN_PARAMS_COUNT;
	conn_params.start_on_notify_cccd_handle = m_cscs.meas_handles.cccd_handle;
	conn_params.disconnect_on_fail = false;
	conn_params.evt_handler = on_conn_params_evt;
	conn_params.error_handler = conn_params_error_handler;
	
	err_code = ble_conn_params_init(&conn_params);
	APP_ERROR_CHECK(err_code);
}
//

static void peer_manager_init()
{
	ret_code_t err_code;
	ble_gap_sec_params_t sec_params;
	
	err_code = pm_init();
	APP_ERROR_CHECK(err_code);
	
	memset(&sec_params, 0, sizeof(sec_params));
	
	sec_params.bond = SEC_PARAM_BOND;
	sec_params.mitm = SEC_PARAM_MITM;
	sec_params.lesc = SEC_PARAM_LESC;
	sec_params.keypress = SEC_PARAM_KEYPRESS;
	sec_params.io_caps = SEC_PARAM_IO_CAPABILITIES;
	sec_params.oob = SEC_PARAM_OOB;
	sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	sec_params.kdist_own.enc = 1;
	sec_params.kdist_own.id = 1;
	sec_params.kdist_peer.enc = 1;
	sec_params.kdist_peer.id = 1;
	
	err_code = pm_sec_params_set(&sec_params);
	APP_ERROR_CHECK(err_code);
	
	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}
//

static void delete_bonds(void)
{
    ret_code_t err_code;

    //NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
//

static void advertising_start(bool erase_bonds)
{
    ret_code_t err_code;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        err_code = ble_advertising_start(&m_adv, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}
//

int main()
{
    bool erase_bonds = false;
	timers_init();
	power_mgmt_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	advertising_init();
	services_init();
	InitBMI();
	conn_params_init();
	peer_manager_init();
	
	
	timers_start();
	advertising_start(erase_bonds);
	while(1)
	{
		//for(int i = 0;i<1000000;++i) asm("NOP");
		//BMI_ReadData();
		idle_state_handle();
	}
}