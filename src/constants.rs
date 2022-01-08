#![allow(dead_code, non_snake_case)]

/// BME680 General config
pub const BME680_POLL_PERIOD_MS: u8 = 10;

/// BME680 unique chip identifier
pub const BME680_CHIP_ID: u8 = 0x61;

/// BME680 field_x related defines
pub const BME680_FIELD_LENGTH: usize = 17;

/// BME680 coefficients related defines
pub const BME680_COEFF_ADDR1_LEN: usize = 25;
pub const BME680_COEFF_ADDR2_LEN: usize = 16;

pub const BME680_SOFT_RESET_CMD: u8 = 0xb6;

/// Register map
/// Other coefficient's address
pub const BME68X_REG_CTRL_GAS_0: u8 = 0x70;
pub const BME68X_REG_CTRL_GAS_1: u8 = 0x71;
pub const BME680_ADDR_RES_HEAT_VAL_ADDR: u8 = 0x00;
pub const BME680_ADDR_RES_HEAT_RANGE_ADDR: u8 = 0x02;
pub const BME680_ADDR_RANGE_SW_ERR_ADDR: u8 = 0x04;
pub const BME680_ADDR_SENS_CONF_START: u8 = 0x5A;
pub const BME680_ADDR_GAS_CONF_START: u8 = 0x64;

pub const BME680_SOFT_RESET_ADDR: u8 = 0xe0;

/*
    Settings
*/
pub const BME68X_ODR_NONE: u8 = 8;

/*
    Bit Masks and Bit Positions
*/
/// General
// Operation Mode
pub const BME68X_MODE_MSK: u8 = 0x03;
// new data
pub const BME68X_NEW_DATA_MSK: u8 = 0x80;
/// Temp
// Oversampling Temp
pub const BME68X_OST_MSK: u8 = 0xe0;
pub const BME68X_OST_POS: u8 = 5;
/// Pressure
// Oversampling Pressure
pub const BME68X_OSP_MSK: u8 = 0x1c;
pub const BME68X_OSP_POS: u8 = 2;
/// Humidity
// Oversampling Humidity
pub const BME68X_OSH_MSK: u8 = 0x07;
pub const BME68X_OSH_POS: u8 = 0;
/// Gas
// Run Gas Measurement
pub const BME68X_RUN_GAS_MSK: u8 = 0x30;
pub const BME68X_RUN_GAS_POS: u8 = 4;
// Number conversion
pub const BME68X_NBCONV_MSK: u8 = 0x0f;
pub const BME68X_NBCONV_POS: u8 = 0;
// IIR filter
pub const BME68X_FILTER_MSK: u8 = 0x1c;
pub const BME68X_FILTER_POS: u8 = 2;
// ODR[3]
pub const BME68X_ODR3_MSK: u8 = 0x80;
pub const BME68X_ODR3_POS: u8 = 7;
// ODR[2:0]
pub const BME68X_ODR20_MSK: u8 = 0xe0;
pub const BME68X_ODR20_POS: u8 = 5;
// Heater Control
pub const BME68X_HCTRL_MSK: u8 = 0x08;
pub const BME68X_HCTRL_POS: u8 = 3;
// Res Heat Range
pub const BME68X_RHRANGE_MSK: u8 = 0x30;
// Range Switching Error
pub const BME68X_RSERROR_MSK: u8 = 0xf0;
// Gas Index
pub const BME68X_GAS_INDEX_MSK: u8 = 0x0f;
// Gas Range
pub const BME68X_GAS_RANGE_MSK: u8 = 0x0f;
// Gas Measurement Valid
pub const BME68X_GASM_VALID_MSK: u8 = 0x20;
// Heater Stability
pub const BME68X_HEAT_STAB_MSK: u8 = 0x10;
// SPI Memory Page
pub const BME68X_MEM_PAGE_MSK: u8 = 0x10;
// Reading a Register in SPI
pub const BME68X_SPI_RD_MSK: u8 = 0x80;
// Writing a Register in SPI
pub const BME68X_SPI_WR_MSK: u8 = 0x7f;
// H1 Calibration Coefficent
pub const BME68X_BIT_H1_DATA_MSK: u8 = 0x0f;

/// Field settings
pub const BME680_FIELD0_ADDR: u8 = 0x1d;

/// Heater settings
pub const BME680_RES_HEAT0_ADDR: u8 = 0x5a;
pub const BME680_GAS_WAIT0_ADDR: u8 = 0x64;
pub const BME680_GAS_IDAC_ADDR: u8 = 0x50;

pub const BME680_DISABLE_GAS_MEAS: u8 = 0x00;
pub const BME680_ENABLE_GAS_MEAS_LOW: u8 = 0x01;
pub const BME680_ENABLE_GAS_MEAS_HIGH: u8 = 0x02;

/// Sensor configuration registers
pub const BME680_CONF_HEAT_CTRL_ADDR: u8 = 0x70;
pub const BME680_CONF_ODR_RUN_GAS_NBC_ADDR: u8 = 0x71;
pub const BME680_CONF_OS_H_ADDR: u8 = 0x72;
pub const BME680_CONF_T_P_MODE_ADDR: u8 = 0x74;
pub const BME680_CONF_ODR_FILT_ADDR: u8 = 0x75;

/// Coefficient's address
pub const BME680_COEFF_ADDR1: u8 = 0x89;
pub const BME680_COEFF_ADDR2: u8 = 0xe1;

/// Chip identifier
pub const BME680_CHIP_ID_ADDR: u8 = 0xd0;
pub const BME680_VARIENT_ID_ADDR: u8 = 0xf0;

pub const BME680_VARIENT_ID_BME680: u8 = 0;
pub const BME680_VARIENT_ID_BME688: u8 = 1;

pub const BME680_SLEEP_MODE: u8 = 0;
pub const BME680_FORCED_MODE: u8 = 1;

pub const BME680_RESET_PERIOD: u8 = 10;

pub const BME680_MODE_MSK: u8 = 0x03;
pub const BME680_RSERROR_MSK: u8 = 0xf0;
pub const BME680_NEW_DATA_MSK: u8 = 0x80;
pub const BME680_GAS_INDEX_MSK: u8 = 0x0f;
pub const BME680_GAS_RANGE_MSK: u8 = 0x0f;
pub const BME680_GASM_VALID_MSK: u8 = 0x20;
pub const BME680_HEAT_STAB_MSK: u8 = 0x10;

/// Buffer length macro declaration
pub const BME680_TMP_BUFFER_LENGTH: usize = 40;
pub const BME680_REG_BUFFER_LENGTH: usize = 5; // !!!: Was 6, but only 5 registers were being set.

pub const GAS_RES_LOOKUP_TABLE_1: [u32; 16] = [
    2147483647u32,
    2147483647u32,
    2147483647u32,
    2147483647u32,
    2147483647u32,
    2126008810u32,
    2147483647u32,
    2130303777u32,
    2147483647u32,
    2147483647u32,
    2143188679u32,
    2136746228u32,
    2147483647u32,
    2126008810u32,
    2147483647u32,
    2147483647u32,
];
pub const GAS_RES_LOOKUP_TABLE_2: [u32; 16] = [
    4096000000u32,
    2048000000u32,
    1024000000u32,
    512000000u32,
    255744255u32,
    127110228u32,
    64000000u32,
    32258064u32,
    16016016u32,
    8000000u32,
    4000000u32,
    2000000u32,
    1,
    500000u32,
    250000u32,
    125000u32,
];


/**
    Apply Masks
*/

// concat 2 u8's into a u16
pub fn BME68X_CONCAT_BYTES(most_sig: u8, least_sig: u8) -> u16 {
    ((most_sig as u16) << 8) | (least_sig as u16)
}

pub fn BME68X_SET_BITS(reg_data: u8, bitmask: u8, bitpos: u8, data: u8) -> u8 {
    (reg_data & !bitmask) | ((data << bitpos) & bitmask)
}

pub fn BME68X_GET_BITS(reg_data: u8, bitmask: u8, bitpos: u8) -> u8 {
    (reg_data & bitmask) >> bitpos
}

pub fn BME68X_SET_BITS_POS_0 (reg_data: u8, bitmask: u8, data: u8) -> u8 {
    (reg_data & !bitmask) | (data & bitmask)
}

pub fn BME68X_GET_BITS_POS_0(reg_data: u8, bitmask: u8) -> u8 {
    reg_data & bitmask
}
