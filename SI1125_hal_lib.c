
#include <SI1125_hal_lib.h>

extern I2C_HandleTypeDef hi2c1;

bool SI1125_begin(void) {
  
  uint8_t id = read8(SI1145_REG_PARTID);
  if (id != 0x45) return false; // look for SI1145
  
  SI1125_reset();
  

    /***********************************/
  // enable UVindex measurement coefficients!
  write8(SI1145_REG_UCOEFF0, 0x29);
  write8(SI1145_REG_UCOEFF1, 0x89);
  write8(SI1145_REG_UCOEFF2, 0x02);
  write8(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);
  // enable interrupt on every sample
  write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);  

/****************************** Prox Sense 1 */

  // program LED current
  write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);



  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


/************************/

  // measurement rate for auto
  write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
  
  // auto run
  write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

  return true;
}

void SI1125_reset() {
  write8(SI1145_REG_MEASRATE0, 0);
  write8(SI1145_REG_MEASRATE1, 0);
  write8(SI1145_REG_IRQEN, 0);
  write8(SI1145_REG_IRQMODE1, 0);
  write8(SI1145_REG_IRQMODE2, 0);
  write8(SI1145_REG_INTCFG, 0);
  write8(SI1145_REG_IRQSTAT, 0xFF);

  write8(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  write8(SI1145_REG_HWKEY, 0x17);
  
  delay(10);
}
// returns the UV index * 100 (divide by 100 to get the index)
uint16_t SI1125_readUV(void) {
 return read16(0x2C); 
}

// returns visible+IR light levels
uint16_t SI1125_readVisible(void) {
 return read16(0x22); 
}

// returns IR light levels
uint16_t SI1125_readIR(void) {
 return read16(0x24); 
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t SI1125_readProx(void) {
 return read16(0x26); 
}

/*********************************************************************/

uint8_t writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);
  
  write8(SI1145_REG_PARAMWR, v);
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8(SI1145_REG_PARAMRD);
}

uint8_t readParam(uint8_t p) {
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

static uint16_t convert_to_uint16(uint8_t bytes[])
{
  return (uint16_t)((bytes[1]<<8) | bytes[0]);
}

uint8_t  read8(uint8_t reg) {
	
	uint8_t buffer;
  if(HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR<<1), &reg, 1, 10000))
    return -1;
  if(HAL_OK != HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDR<<1), &buffer, 1, 10000))
    return -1;
  
    return buffer;
}

uint16_t read16(uint8_t a) {
	uint16_t code;
	uint8_t buffer[2];
	if(HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR<<1), &a, 1, 10000))
		return -1;

	if(HAL_OK != HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDR<<1), buffer, 2, 10000))
		return -1;
  
	code = convert_to_uint16(buffer);

	return code;
}

void write8(uint8_t reg, uint8_t val) {

  uint8_t buff[2] = {reg,val}
  if(HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDR<<1), buff, 2, 10000))
		return -1;
  
}