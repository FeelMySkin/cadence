#include "bmi_controller.h"

static const nrf_drv_twi_t hi2c = NRF_DRV_TWI_INSTANCE(0);

static void BMI_Exists();
static void BMI_ReadBytes(uint8_t reg, uint8_t len);
static void BMI_WriteBytes(uint8_t reg, uint8_t *data, uint8_t len);
static void BMI_ProcessReceived();
static void BMI_ProcessGyro(uint8_t* start);

uint8_t recv_data[10];
uint8_t write_data[10];

static struct MMU_Data gyro;

enum BMI_StateMachine
{
	BMI_IDLE,
	BMI_ACTIVE
};

struct BMI_Handler
{
	enum BMI_StateMachine state:2;
	bool is_found:1;
	bool is_configured:1;
	
} static bmi_control;

void i2c_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
	switch (p_event ->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
			if(p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
			{
				bmi_control.state = BMI_IDLE;
			}
			if(p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
			{
				BMI_ProcessReceived();
				bmi_control.state = BMI_IDLE;
			}
		break;
			

		default:
			bmi_control.state = BMI_IDLE;
		break;
	}
}

void BMI_Init(struct BMI_InitStruct* bmi)
{

	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_conf = {
		.scl = bmi->scl_pin,
		.sda = bmi->sda_pin,
		.frequency = NRF_DRV_TWI_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};
	err_code = nrf_drv_twi_init(&hi2c,&twi_conf,i2c_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&hi2c);
	
	bmi_control.state = BMI_IDLE;
}

bool BMI_Setup()
{
	if(!bmi_control.is_found)
	{
		BMI_ReadBytes(BMI_WHO_AM_I_REG,1);
		return false;
	}
	
	if(!bmi_control.is_configured)
	{
		/*Write GYRO Config*/
		uint8_t reg_writer = (8)/*100Hz*/ | (2<<4)/*Normal filter?*/;
		BMI_WriteBytes(BMI_GYRO_CONF_REG,&reg_writer,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		/*Check errors*/
		BMI_ReadBytes(BMI_ERR_REG,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		if(recv_data[0] != 0x00) return false;
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		/*Check GYRO Config*/
		BMI_ReadBytes(BMI_GYRO_CONF_REG,2);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		if(recv_data[0] != reg_writer) return false;
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		/*Write GYRO Range*/
		reg_writer = 1/*+-1000 deg/s => 30.5mdeg/s / LSB*/;
		BMI_WriteBytes(BMI_GYRO_RANGE_REG,&reg_writer,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		/*Check GYRO Range*/
		BMI_ReadBytes(BMI_GYRO_RANGE_REG,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		if(recv_data[0] != reg_writer) return false;
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		reg_writer = 0x15; //GYRO Normal Mode command.
		BMI_WriteBytes(BMI_CMD_REG,&reg_writer,1);
		for(int i = 0;i<1000000;++i) asm("NOP");
		
		return true;
		
		
	}
	return true;
}

void BMI_ReadData()
{
	BMI_ReadBytes(BMI_STATUS_REG,1);
	while(bmi_control.state != BMI_IDLE) asm("NOP");
	for(int i = 0;i<1000000;++i) asm("NOP");
	
	if((recv_data[0]&(1<<6)) == 0)
	{
		return;
	}
	
	BMI_ReadBytes(BMI_DATA8_REG,6);
}

void BMI_ProcessReceived()
{
	switch(write_data[0])
	{
		case BMI_WHO_AM_I_REG:
			if(recv_data[0] == BMI_ID)
			{
				bmi_control.is_found = true;
			}
		break;
			
		case BMI_DATA8_REG:
			BMI_ProcessGyro(recv_data);
		break;
			
		default:
			break;
	}
}

void BMI_CalibrateGyro()
{
	while(bmi_control.state != BMI_IDLE) asm("NOP");
	uint8_t dater = 1<<6; //Enable FOC_Gyro calibration (Fast offset compensation)
	BMI_WriteBytes(BMI_FOC_CONF_REG,&dater,1);
	while(bmi_control.state != BMI_IDLE) asm("NOP");
	for(int i = 0;i<1000000;++i) asm("NOP");
	
	dater = 0x03; //Statrt FOC
	BMI_WriteBytes(BMI_CMD_REG,&dater,1);
	while(bmi_control.state != BMI_IDLE) asm("NOP");
	for(int i = 0;i<1000000;++i) asm("NOP");
	
	while(1)
	{
		BMI_ReadBytes(BMI_STATUS_REG,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		for(int i = 0;i<1000000;++i) asm("NOP");
		if(recv_data[0] & (1<<3)) break;
	}
	dater = 0xA0; //Save NVM command;
	BMI_WriteBytes(BMI_CMD_REG,&dater,1);
	
	while(1)
	{
		BMI_ReadBytes(BMI_STATUS_REG,1);
		while(bmi_control.state != BMI_IDLE) asm("NOP");
		for(int i = 0;i<1000000;++i) asm("NOP");
		if(recv_data[0] & (1<<4)) break;
	}
}

void BMI_ProcessGyro(uint8_t* start)
{
	gyro.x = ((int16_t)(start[0] | (start[1]<<8)))*30.5/1000.0;
	gyro.y = ((int16_t)(start[2] | (start[3]<<8)))*30.5/1000.0;
	gyro.z = ((int16_t)(start[4] | (start[5]<<8)))*30.5/1000.0;
}

void BMI_ReadBytes(uint8_t reg, uint8_t len)
{
	if(bmi_control.state != BMI_IDLE) return;
	ret_code_t err_code;
	write_data[0] = reg;

	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(BMI_ADDR,write_data,1,recv_data,len);
	err_code = nrf_drv_twi_xfer(&hi2c,&xfer,0);
	
	bmi_control.state = BMI_ACTIVE;

}

void BMI_WriteBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
	if(bmi_control.state != BMI_IDLE) return;
	ret_code_t err_code;
	write_data[0] = reg;
	for(int i = 0;i<len;++i) write_data[i+1] = data[i];

	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TX(BMI_ADDR,write_data,len+1);
	err_code = nrf_drv_twi_xfer(&hi2c,&xfer,0);
	
	bmi_control.state = BMI_ACTIVE;
}