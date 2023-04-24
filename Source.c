#include "Source.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_twi.h"
#include "nrf.h"

BLE_BAS_DEF(m_bas); //battery service instance
BLE_CSCS_DEF(m_cscs); //Cycling speed and cadense service instance
NRF_BLE_GATT_DEF(m_gatt); //GATT module instance
NRF_BLE_QWR_DEF(m_qwr); //Queued Write Module context
BLE_ADVERTISING_DEF(m_adv); //Advertising module instance
APP_TIMER_DEF(m_battery_tim); //Battery timer
APP_TIMER_DEF(m_cscs_tim); //CSC measure timer


bool sending = false;
bool sent = false;
bool receiving = false;
bool received = false;
uint8_t recv_byte;


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

static void InitBMI()
{
	struct BMI_InitStruct bmi;
	bmi.scl_pin = I2C0_SCL_PIN;
	bmi.sda_pin = I2C0_SDA_PIN;
	bmi.i2c_instance = 0;

	BMI_Init(&bmi);
	while(!BMI_Setup()) asm("NOP");
}

int main()
{
	timers_init();
	InitBMI();
	//BMI_CalibrateGyro();
	while(1)
	{
		for(int i = 0;i<1000000;++i) asm("NOP");
		BMI_ReadData();
		//idle_state_handle();
	}
}