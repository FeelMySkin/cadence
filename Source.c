#include "Source.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_twi.h"

BLE_BAS_DEF(m_bas); //battery service instance
BLE_CSCS_DEF(m_cscs); //Cycling speed and cadense service instance
NRF_BLE_GATT_DEF(m_gatt); //GATT module instance
NRF_BLE_QWR_DEF(m_qwr); //Queued Write Module context
BLE_ADVERTISING_DEF(m_adv); //Advertising module instance
APP_TIMER_DEF(m_battery_tim); //Battery timer
APP_TIMER_DEF(m_cscs_tim); //CSC measure timer

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	
	
    //battery_level_update();
}

static void csc_meas_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    ble_cscs_meas_t cscs_measurement;

    UNUSED_PARAMETER(p_context);

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
}

static void idle_state_handle(void)
{
    //if (NRF_LOG_PROCESS() == false)
   // {
        nrf_pwr_mgmt_run();
   // }
}

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event ->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
			if(p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
			{
				//do something with readed data.
			}
		break;

		default:
		break;
	}
}

static void init_twi()
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_conf = {
		.scl = 20,
		.sda = 21,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};
	err_code = nrf_drv_twi_init(&m_twi,&twi_conf,twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

static void read_data()
{
	//ret_code_t err_code = nrf_drv_twi_rx(&m_twi,/*DEVICE ADDR*/, /*WHERE TO READ*/,/*SIZEOF(READ)*/);
	//APP_ERROR_CHECK(err_code);
}

int main()
{
	timers_init();
	init_twi();
	while(1)
	{
		idle_state_handle();
	}
}