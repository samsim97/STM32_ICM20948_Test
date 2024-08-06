#pragma once

#include <Sensors/Altimeter/IAltimeterDriver.hpp>
#include <HardwareDriver/BMP388/BMP388Defines.hpp>

#include "stm32f4xx_hal.h"
#include <string.h>
#include <cmath>

//#include <HardwareDriver/Bmp388/bmp3.h>

class BMP388 : public IAltimeterDriver
{
public:
	BMP388(I2C_HandleTypeDef* i2chandle);
	~BMP388() {};

	struct bmp3_dev dev;

	void init();
	void readAltimeter();
	AltimeterValues getAltimeterValues();

private:
	I2C_HandleTypeDef* i2cHandle;

	AltimeterValues altimeterValues;

	// REFACTOR
	uint8_t GTXBuffer[512], GRXBuffer[2048];
	uint8_t dev_addr = 0;
	int8_t rslt;
	uint8_t loop = 0;
	uint16_t settings_sel;

	struct bmp3_data data = { 0 };
	struct bmp3_settings settings = { 0 };
	struct bmp3_status status = { { 0 } };

	int8_t SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr);

	int8_t SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr);

	void bmp3_delay_us(uint32_t period, void *intf_ptr);

	int8_t bmp3_init(struct bmp3_dev *dev);
	int8_t null_ptr_check(const struct bmp3_dev *dev);
	int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev);
	int8_t get_calib_data(struct bmp3_dev *dev);
	void parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev);
	int8_t bmp3_soft_reset(struct bmp3_dev *dev);
	int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bmp3_dev *dev);
	void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint32_t len);
	int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data, struct bmp3_dev *dev);
	void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data);
	int8_t compensate_data(uint8_t sensor_comp,
	                              const struct bmp3_uncomp_data *uncomp_data,
	                              struct bmp3_data *comp_data,
	                              struct bmp3_calib_data *calib_data);
	int8_t compensate_temperature(double *temperature,
	                                     const struct bmp3_uncomp_data *uncomp_data,
	                                     struct bmp3_calib_data *calib_data);
	int8_t compensate_pressure(double *pressure,
	                                  const struct bmp3_uncomp_data *uncomp_data,
	                                  const struct bmp3_calib_data *calib_data);
	float pow_bmp3(double base, uint8_t power);
	int8_t bmp3_get_status(struct bmp3_status *status, struct bmp3_dev *dev);
	int8_t get_sensor_status(struct bmp3_status *status, struct bmp3_dev *dev);
	int8_t get_int_status(struct bmp3_status *status, struct bmp3_dev *dev);
	int8_t get_err_status(struct bmp3_status *status, struct bmp3_dev *dev);
	int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_settings *settings, struct bmp3_dev *dev);
	int8_t bmp3_set_op_mode(struct bmp3_settings *settings, struct bmp3_dev *dev);
	uint8_t are_settings_changed(uint32_t sub_settings, uint32_t desired_settings);
	int8_t set_pwr_ctrl_settings(uint32_t desired_settings,
	                                    const struct bmp3_settings *settings,
	                                    struct bmp3_dev *dev);
	int8_t set_odr_filter_settings(uint32_t desired_settings, struct bmp3_settings *settings, struct bmp3_dev *dev);
	int8_t set_int_ctrl_settings(uint32_t desired_settings,
	                                    const struct bmp3_settings *settings,
	                                    struct bmp3_dev *dev);
	int8_t set_advance_settings(uint32_t desired_settings, const struct bmp3_settings *settings,
	                                   struct bmp3_dev *dev);
	int8_t bmp3_get_op_mode(uint8_t *op_mode, struct bmp3_dev *dev);
	int8_t put_device_to_sleep(struct bmp3_dev *dev);
	int8_t set_normal_mode(struct bmp3_settings *settings, struct bmp3_dev *dev);
	int8_t write_power_mode(const struct bmp3_settings *settings, struct bmp3_dev *dev);
	void fill_osr_data(uint32_t desired_settings,
	                          uint8_t *addr,
	                          uint8_t *reg_data,
	                          uint8_t *len,
	                          const struct bmp3_settings *settings);
	void fill_odr_data(uint8_t *addr, uint8_t *reg_data, uint8_t *len, struct bmp3_settings *settings);
	void fill_filter_data(uint8_t *addr, uint8_t *reg_data, uint8_t *len, const struct bmp3_settings *settings);
	int8_t validate_osr_and_odr_settings(const struct bmp3_settings *settings);
	int8_t verify_meas_time_and_odr_duration(uint32_t meas_t, uint32_t odr_duration);
	uint32_t calculate_temp_meas_time(const struct bmp3_settings *settings);
	uint32_t calculate_press_meas_time(const struct bmp3_settings *settings);
	int8_t validate_normal_mode_settings(struct bmp3_settings *settings, struct bmp3_dev *dev);
	int8_t get_odr_filter_settings(struct bmp3_settings *settings, struct bmp3_dev *dev);
	void  parse_odr_filter_settings(const uint8_t *reg_data, struct bmp3_odr_filter_settings *settings);
};
