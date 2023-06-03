//! This crate is a pure Rust implementation for the BME680 environmental sensor.
//! The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.
//!
//! The library uses the embedded-hal crate to abstract reading and writing via I²C.
//! In the examples you can find a demo how to use the library in Linux using the linux-embedded-hal crate (e.g. on a RPI).
//! ```no_run

//! extern crate bme680;
//! extern crate embedded_hal;
//! // Note that you'll have to import your board crates types corresponding to
//! // Delay and I2cdev.
//!
//! use bme680::*;
//! use embedded_hal::blocking::i2c;
//! use hal::*;
//! use std::result;
//! use std::time::Duration;
//! use std::thread::sleep;
//!
//! # mod hal {
//! #   use super::*;
//! #   use embedded_hal::blocking::delay;
//! #
//! #   #[derive(Debug)]
//! #   pub struct Delay {}
//! #
//! #   impl delay::DelayMs<u8> for Delay {
//! #       fn delay_ms(&mut self, _ms: u8) {}
//! #   }
//! #
//! #   #[derive(Debug)]
//! #   pub enum I2CError {}
//! #
//! #   pub struct I2cdev {}
//! #
//! #   impl i2c::Write for I2cdev {
//! #       type Error = I2CError;
//! #
//! #       fn write<'w>(&mut self, addr: u8, bytes: &'w [u8]) -> result::Result<(), Self::Error> {
//! #           Ok(())
//! #       }
//! #   }
//! #
//! #   impl i2c::Read for I2cdev {
//! #       type Error = I2CError;
//! #
//! #       fn read<'w>(&mut self, addr: u8, bytes: &'w mut [u8]) -> result::Result<(), Self::Error> {
//! #           Ok(())
//! #       }
//! #   }
//! # }
//!
//! fn main() -> result::Result<(), Error<<hal::I2cdev as i2c::Read>::Error, <hal::I2cdev as i2c::Write>::Error>>
//! {
//!     // Initialize device
//!     let i2c = I2cdev {};        // Your I2C device construction will look different, perhaps using I2cdev::new(..)
//!     let mut delayer = Delay {}; // Your Delay construction will look different, perhaps using Delay::new(..)
//!     let mut dev = Bme680::init(i2c, &mut delayer, I2CAddress::Primary)?;
//!     let settings = SettingsBuilder::new()
//!         .with_humidity_oversampling(OversamplingSetting::OS2x)
//!         .with_pressure_oversampling(OversamplingSetting::OS4x)
//!         .with_temperature_oversampling(OversamplingSetting::OS8x)
//!         .with_temperature_filter(IIRFilterSize::Size3)
//!         .with_gas_measurement(Duration::from_millis(150), 320, 25)
//!         .with_run_gas(true)
//!         .build();
//!     dev.set_sensor_settings(&mut delayer, settings)?;
//!     let profile_duration = dev.get_profile_dur(&settings.0)?;
//!
//!     // Read sensor data
//!     dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)?;
//!     sleep(profile_duration);
//!     let (data, _state) = dev.get_sensor_data(&mut delayer)?;
//!
//!     println!("Temperature {}°C", data.temperature_celsius());
//!     println!("Pressure {}hPa", data.pressure_hpa());
//!     println!("Humidity {}%", data.humidity_percent());
//!     println!("Gas Resistence {}Ω", data.gas_resistance_ohm());
//!
//!     Ok(())
//! }
//! ```

#![no_std]
#![forbid(unsafe_code)]

pub use self::settings::{
    DesiredSensorSettings, GasSett, IIRFilterSize, OversamplingSetting, SensorSettings, Settings,
    SettingsBuilder, TphSett,
};

mod calc;
mod settings;
mod constants;

use crate::constants::*;
use crate::calc::Calc;
use crate::hal::delay::DelayUs;
use crate::hal::i2c::I2c;

use core::time::Duration;
use core::{marker::PhantomData, result};
use embedded_hal as hal;
use defmt::{info, error, debug};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<I> {
    ///
    /// aka BME680_E_COM_FAIL
    ///
    I2C(I),
    Delay,
    ///
    /// aka BME680_E_DEV_NOT_FOUND
    ///
    DeviceNotFound,
    ///
    /// aka BME680_E_INVALID_LENGTH
    ///
    InvalidLength,
    ///
    /// Warning aka BME680_W_DEFINE_PWR_MODE
    ///
    DefinePwrMode,
    ///
    /// Warning aka BME680_W_DEFINE_PWR_MODE
    ///
    NoNewData,
    ///
    /// Warning Boundary Check
    ///
    BoundaryCheckFailure(&'static str),
}

/// Abbreviates `std::result::Result` type
pub type Result<T, I> = result::Result<T, Error<I>>;

///
/// Chip Varient ID
///
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum VarientId {
    BME680,
    BME688,
}

impl Default for VarientId {
    fn default() -> Self { VarientId::BME680 }
}

impl VarientId {
    fn from(varient_id: u8) -> Self {
        match varient_id {
            BME680_VARIENT_ID_BME680 => VarientId::BME680,
            BME680_VARIENT_ID_BME688 => VarientId::BME688,
            _ => panic!("Unknown varient id: {}", varient_id),
        }
    }

    fn value(&self) -> u8 {
        match self {
            VarientId::BME680 => BME680_VARIENT_ID_BME680,
            VarientId::BME688 => BME680_VARIENT_ID_BME688,
        }
    }
}

///
/// Power mode settings
///
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PowerMode {
    SleepMode,
    ForcedMode,
}

impl PowerMode {
    // TODO replace with TryFrom once stabilized
    fn from(power_mode: u8) -> Self {
        match power_mode {
            BME680_SLEEP_MODE => PowerMode::SleepMode,
            BME680_FORCED_MODE => PowerMode::ForcedMode,
            _ => panic!("Unknown power mode: {}", power_mode),
        }
    }

    fn value(&self) -> u8 {
        match self {
            PowerMode::SleepMode => BME680_SLEEP_MODE,
            PowerMode::ForcedMode => BME680_FORCED_MODE,
        }
    }
}

///
/// I2C Slave Address
/// To determine the slave address of your device you can use `i2cdetect -y 1` on linux.
/// The 7-bit device address is 111011x. The 6 MSB bits are fixed.
/// The last bit is changeable by SDO value and can be changed during operation.
/// Connecting SDO to GND results in slave address 1110110 (0x76); connecting it to V DDIO results in slave
/// address 1110111 (0x77), which is the same as BMP280’s I2C address.
///
#[derive(Debug, Clone, Copy)]
pub enum I2CAddress {
    /// Primary Slave Address 0x76
    Primary,
    /// Secondary Slave Address 0x77
    Secondary,
    /// Alternative address
    Other(u8),
}

impl I2CAddress {
    pub fn addr(&self) -> u8 {
        match &self {
            I2CAddress::Primary => 0x76u8,
            I2CAddress::Secondary => 0x77u8,
            I2CAddress::Other(addr) => *addr,
        }
    }
}

impl Default for I2CAddress {
    fn default() -> I2CAddress {
        I2CAddress::Primary
    }
}

/// Calibration data used during initalization
#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct CalibData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: u8,
}

impl Clone for CalibData {
    fn clone(&self) -> Self {
        *self
    }
}

/// Contains read sensors values  e.g. temperature, pressure, humidity etc.
#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct FieldData {
    /// Contains new_data, gasm_valid & heat_stab
    status: u8,
    /// Index of heater profile used
    gas_index: u8,
    /// Measurement index
    meas_index: u8,
    temperature: i16,
    pressure: u32,
    humidity: u32,
    gas_resistance: u32,
}

impl Clone for FieldData {
    fn clone(&self) -> Self {
        *self
    }
}

impl FieldData {
    /// Temperature in degree celsius (°C)
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature as f32 / 100f32
    }

    /// Pressure in hectopascal (hPA)
    pub fn pressure_hpa(&self) -> f32 {
        self.pressure as f32 / 100f32
    }

    /// Humidity in % relative humidity
    pub fn humidity_percent(&self) -> f32 {
        self.humidity as f32 / 1000f32
    }

    pub fn gas_resistance_ohm(&self) -> u32 {
        self.gas_resistance
    }

    /// Whether a real (and not a dummy) gas reading was performed.
    pub fn gas_valid(&self) -> bool {
        self.status & BME680_GASM_VALID_MSK != 0
    }

    /// Whether the heater target temperature for the gas reading was reached.
    ///
    /// If this values is `false`, the heating duration was likely too short or
    /// the target temperature too high.
    pub fn heat_stable(&self) -> bool {
        self.status & BME680_HEAT_STAB_MSK != 0
    }
}

/// Shows if new data is available
#[derive(PartialEq, Debug)]
pub enum FieldDataCondition {
    ///
    /// Data changed since last read
    ///
    NewData,
    ///
    /// Data has not changed since last read
    ///
    Unchanged,
}

struct I2CUtil {}

impl I2CUtil {
    pub fn read_byte<I2C>(
        i2c: &mut I2C,
        dev_id: u8,
        reg_addr: u8,
    ) -> Result<u8, <I2C>::Error>
    where
        I2C: I2c,
    {
        let mut buf = [0; 1];

        i2c.write(dev_id, &[reg_addr]).map_err(Error::I2C)?;

        match i2c.read(dev_id, &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(Error::I2C(e))?,
        }
    }

    pub fn read_bytes<I2C>(
        i2c: &mut I2C,
        dev_id: u8,
        reg_addr: u8,
        buf: &mut [u8],
    ) -> Result<(), <I2C>::Error>
    where
        I2C: I2c,
    {
        i2c.write(dev_id, &[reg_addr]).map_err(Error::I2C)?;

        match i2c.read(dev_id, buf) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::I2C(e)),
        }
    }
}

/// Driver for the BME680 environmental sensor
#[repr(C)]
pub struct Bme680<I2C, D> {
    i2c: I2C,
    delay: PhantomData<D>,
    dev_id: I2CAddress,
    calib: CalibData,
    // TODO remove ? as it may not reflect the state of the device
    tph_sett: TphSett,
    // TODO remove ? as it may not reflect the state of the device
    gas_sett: GasSett,
    // TODO remove ? as it may not reflect the state of the device
    power_mode: PowerMode,
    // TODO figure out how default works
    varient_id: VarientId,
}

fn boundary_check<I2C>(
    value: Option<u8>,
    value_name: &'static str,
    max: u8,
) -> Result<u8, <I2C>::Error>
where
    I2C: I2c,
{
    let value = value.ok_or(Error::BoundaryCheckFailure(value_name))?;
    if value > max {
        const MAX: &str = "Boundary check, value exceeds minimum";
        error!("{}, value name: {}", MAX, value_name);
        return Err(Error::BoundaryCheckFailure(MAX));
    }
    Ok(value)
}

impl<I2C, D> Bme680<I2C, D>
where
    D: DelayUs,
    I2C: I2c,
{
    pub fn soft_reset(
        i2c: &mut I2C,
        delay: &mut D,
        dev_id: I2CAddress,
    ) -> Result<(), <I2C>::Error> {
        let tmp_buff: [u8; 2] = [BME680_SOFT_RESET_ADDR, BME680_SOFT_RESET_CMD];

        i2c.write(dev_id.addr(), &tmp_buff)
            .map_err(Error::I2C)?;

        delay
            .delay_ms(BME680_RESET_PERIOD as u32);
        Ok(())
    }

    pub fn init(
        mut i2c: I2C,
        delay: &mut D,
        dev_id: I2CAddress,
    ) -> Result<Bme680<I2C, D>, <I2C>::Error> {
        Bme680::soft_reset(&mut i2c, delay, dev_id)?;

        debug!("Reading chip id");
        /* Soft reset to restore it to default values*/
        let chip_id = I2CUtil::read_byte::<I2C>(&mut i2c, dev_id.addr(), BME680_CHIP_ID_ADDR)?;
        let varient_id_raw = I2CUtil::read_byte::<I2C>(&mut i2c, dev_id.addr(), BME680_VARIENT_ID_ADDR)?;
        debug!("Chip id: {}", chip_id);

        if chip_id == BME680_CHIP_ID {
            debug!("Reading calib data");
            let calib = Bme680::<I2C, D>::get_calib_data::<I2C>(&mut i2c, dev_id)?;
            debug!("Calib data {:?}", calib);
            let dev = Bme680 {
                varient_id: VarientId::from(varient_id_raw),
                i2c,
                delay: PhantomData,
                dev_id,
                calib,
                power_mode: PowerMode::ForcedMode,
                tph_sett: Default::default(),
                gas_sett: Default::default(),
            };
            info!("Finished device init");
            Ok(dev)
        } else {
            error!("Device does not match chip id {}", BME680_CHIP_ID);
            Err(Error::DeviceNotFound)
        }
    }

    fn bme680_set_regs(
        &mut self,
        reg: &[(u8, u8)],
    ) -> Result<(), <I2C>::Error> {
        if reg.is_empty() || reg.len() > (BME680_TMP_BUFFER_LENGTH / 2) as usize {
            return Err(Error::InvalidLength);
        }

        for (reg_addr, reg_data) in reg {
            let tmp_buff: [u8; 2] = [*reg_addr, *reg_data];
            debug!(
                "Setting register reg: {:?} tmp_buf: {:?}",
                reg_addr, tmp_buff
            );
            self.i2c
                .write(self.dev_id.addr(), &tmp_buff)
                .map_err(Error::I2C)?;
        }

        Ok(())
    }

    /// Set the settings to be used during the sensor measurements
    pub fn set_sensor_settings(
        &mut self,
        delay: &mut D,
        settings: Settings,
    ) -> Result<(), <I2C>::Error> {
        let (sensor_settings, desired_settings) = settings;
        let tph_sett = sensor_settings.tph_sett;
        let gas_sett = sensor_settings.gas_sett;

        let mut reg: [(u8, u8); BME680_REG_BUFFER_LENGTH] = [(0, 0); BME680_REG_BUFFER_LENGTH];
        let intended_power_mode = self.power_mode;


        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            debug!("GAS_MEAS_SEL: true");
            self.set_gas_config(gas_sett)?;
        }

        let power_mode = self.power_mode;
        self.set_sensor_mode(delay, power_mode)?;

        let mut element_index = 0;
        // Selecting the filter
        if desired_settings.contains(DesiredSensorSettings::FILTER_SEL) {
            let mut data = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_ODR_FILT_ADDR)?;

            debug!("FILTER_SEL: true");
            let filter_setting = tph_sett.filter.unwrap_or(IIRFilterSize::Size0) as u8;
            data = DesiredSensorSettings::FILTER_SEL.set_bits(data, filter_setting).unwrap();
            reg[element_index] = (BME680_CONF_ODR_FILT_ADDR, data);
            element_index += 1;
        }

        // Selecting heater T,P oversampling for the sensor
        if desired_settings.contains(DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL)
        {
            let mut data = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;

            if desired_settings.contains(DesiredSensorSettings::OST_SEL) {
                debug!("OST_SEL: true");
                let tph_sett_os_temp = boundary_check::<I2C>(tph_sett.os_temp.map(|x| x as u8), "TphSett.os_temp", OversamplingSetting::OS16x as u8)?;
                data = DesiredSensorSettings::OST_SEL.set_bits(data, tph_sett_os_temp).unwrap();
            }

            if desired_settings.contains(DesiredSensorSettings::OSP_SEL) {
                debug!("OSP_SEL: true");
                let tph_sett_os_pres = boundary_check::<I2C>(tph_sett.os_pres.map(|x| x as u8), "TphSett.os_pres", OversamplingSetting::OS16x as u8)?;
                data = DesiredSensorSettings::OSP_SEL.set_bits(data, tph_sett_os_pres).unwrap();
            }
            reg[element_index] = (BME680_CONF_T_P_MODE_ADDR, data);
            element_index += 1;
        }

        // Selecting humidity oversampling for the sensor
        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            debug!("OSH_SEL: true");
            let tph_sett_os_hum = boundary_check::<I2C>(tph_sett.os_hum.map(|x| x as u8), "TphSett.os_hum", OversamplingSetting::OS16x as u8)?;
            let mut data = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_OS_H_ADDR)?;
            data = DesiredSensorSettings::OSH_SEL.set_bits(data, tph_sett_os_hum).unwrap();
            reg[element_index] = (BME680_CONF_OS_H_ADDR, data);
            element_index += 1;
        }

        if desired_settings.contains(DesiredSensorSettings::HCNTRL_SEL) {
            debug!("HCNTRL_SEL: true");
            let gas_sett_heatr_ctrl = boundary_check::<I2C>(gas_sett.heatr_ctrl, "GasSett.heatr_ctrl", 0x8u8)?;
            let mut data = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_HEAT_CTRL_ADDR)?;
            data = DesiredSensorSettings::HCNTRL_SEL.set_bits(data, gas_sett_heatr_ctrl).unwrap();
            reg[element_index] = (BME680_CONF_HEAT_CTRL_ADDR, data);
            element_index += 1;
        }

        // Selecting the runGas and NB conversion settings for the sensor
        if desired_settings.contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL) {
            debug!("RUN_GAS_SEL: true");
            let mut data = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_ODR_RUN_GAS_NBC_ADDR)?;
            // run_gas
            let gas_sett_rn = if self.varient_id == VarientId::BME688 { BME680_ENABLE_GAS_MEAS_HIGH } else { BME680_ENABLE_GAS_MEAS_LOW };
            data = DesiredSensorSettings::RUN_GAS_SEL.set_bits(data, gas_sett_rn).unwrap();

            // number conversion
            let gas_sett_nb_conv = boundary_check::<I2C>(Some(gas_sett.nb_conv), "GasSett.nb_conv", 10)?;
            data = DesiredSensorSettings::NBCONV_SEL.set_bits(data, gas_sett_nb_conv).unwrap();

            reg[element_index] = (BME680_CONF_ODR_RUN_GAS_NBC_ADDR, data);
            element_index += 1;
        }

        self.bme680_set_regs(&reg[0..element_index])?;

        // Restore previous intended power mode
        self.power_mode = intended_power_mode;
        self.tph_sett = tph_sett;
        Ok(())
    }

    /// Retrieve settings from sensor registers
    ///
    /// # Arguments
    ///
    /// * `desired_settings` - Settings to be retrieved. Setting values may stay `None` if not retrieved.
    pub fn get_sensor_settings(
        &mut self,
        desired_settings: DesiredSensorSettings,
    ) -> Result<SensorSettings, <I2C>::Error> {
        let reg_addr: u8 = BME68X_REG_CTRL_GAS_0;
        let mut data_array: [u8; BME680_REG_BUFFER_LENGTH] = [0; BME680_REG_BUFFER_LENGTH];
        let mut sensor_settings: SensorSettings = Default::default();
        sensor_settings.tph_sett.temperature_offset = self.tph_sett.temperature_offset;

        I2CUtil::read_bytes(&mut self.i2c, self.dev_id.addr(), reg_addr, &mut data_array)?;

        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            sensor_settings.gas_sett = self.get_gas_config()?;
        }

        if desired_settings.contains(DesiredSensorSettings::FILTER_SEL) {
            let filter = DesiredSensorSettings::FILTER_SEL.get_bits(data_array[4]).unwrap();
            sensor_settings.tph_sett.filter = Some(IIRFilterSize::from_u8(filter));
        }

        if desired_settings.contains(DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL)
        {
            let os_temp: u8 = DesiredSensorSettings::OST_SEL.get_bits(data_array[3]).unwrap();
            let os_pres: u8 = DesiredSensorSettings::OSP_SEL.get_bits(data_array[3]).unwrap();
            sensor_settings.tph_sett.os_temp = Some(OversamplingSetting::from_u8(os_temp));
            sensor_settings.tph_sett.os_pres = Some(OversamplingSetting::from_u8(os_pres));
        }

        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            let os_hum: u8 = DesiredSensorSettings::OSH_SEL.get_bits(data_array[1]).unwrap();
            sensor_settings.tph_sett.os_hum = Some(OversamplingSetting::from_u8(os_hum));
        }

        if desired_settings.contains(DesiredSensorSettings::HCNTRL_SEL) {
            sensor_settings.gas_sett.heatr_ctrl = DesiredSensorSettings::HCNTRL_SEL.get_bits(data_array[0]);
        }

        if desired_settings.contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL) {
            sensor_settings.gas_sett.nb_conv = DesiredSensorSettings::NBCONV_SEL.get_bits(data_array[1]).unwrap();
            sensor_settings.gas_sett.run_gas_measurement = DesiredSensorSettings::RUN_GAS_SEL.get_bits(data_array[1]).unwrap() > 0;
        }

        Ok(sensor_settings)
    }

    /// Retrieve current sensor power mode via registers
    pub fn get_varient_id(
        &mut self,
    ) -> Result<VarientId, <I2C>::Error> {
        let varient_id = I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_VARIENT_ID_ADDR)?;
        Ok(VarientId::from(varient_id))
    }

    /// Set the sensor into a certain power mode
    ///
    /// # Arguments
    ///
    /// * `target_power_mode` - Desired target power mode
    pub fn set_sensor_mode(
        &mut self,
        delay: &mut D,
        target_power_mode: PowerMode,
    ) -> Result<(), <I2C>::Error> {
        let mut tmp_pow_mode: u8;
        let mut current_power_mode: PowerMode;

        // Call repeatedly until in sleep
        loop {
            tmp_pow_mode =
                I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;

            // Put to sleep before changing mode
            current_power_mode = PowerMode::from(tmp_pow_mode & BME680_MODE_MSK);

            debug!("Current power mode: {:?}", current_power_mode);

            if current_power_mode != PowerMode::SleepMode {
                // Set to sleep
                tmp_pow_mode &= !BME680_MODE_MSK;
                debug!("Setting to sleep tmp_pow_mode: {}", tmp_pow_mode);
                self.bme680_set_regs(&[(BME680_CONF_T_P_MODE_ADDR, tmp_pow_mode)])?;
                delay
                    .delay_ms(BME680_POLL_PERIOD_MS as u32);
            } else {
                // TODO do while in Rust?
                break;
            }
        }

        // Already in sleep
        if target_power_mode != PowerMode::SleepMode {
            tmp_pow_mode = tmp_pow_mode & !BME680_MODE_MSK | target_power_mode.value();
            debug!("Already in sleep Target power mode: {}", tmp_pow_mode);
            self.bme680_set_regs(&[(BME680_CONF_T_P_MODE_ADDR, tmp_pow_mode)])?;
        }
        Ok(())
    }

    /// Retrieve current sensor power mode via registers
    pub fn get_sensor_mode(
        &mut self,
    ) -> Result<PowerMode, <I2C>::Error> {
        let regs =
            I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;
        let mode = regs & BME680_MODE_MSK;
        Ok(PowerMode::from(mode))
    }

    pub fn bme680_set_profile_dur(&mut self, tph_sett: TphSett, duration: Duration) {
        let os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        // TODO check if the following unwrap_ors do not change behaviour
        // TODO replace once https://github.com/rust-lang/rust/pull/50167 has been merged
        const MILLIS_PER_SEC: u64 = 1_000;
        const NANOS_PER_MILLI: u64 = 1_000_000;
        let millis = (duration.as_secs() as u64 * MILLIS_PER_SEC)
            + (duration.subsec_nanos() as u64 / NANOS_PER_MILLI);

        let mut meas_cycles = os_to_meas_cycles
            [tph_sett.os_temp.unwrap_or(OversamplingSetting::OSNone) as usize]
            as u64;
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[tph_sett.os_pres.unwrap_or(OversamplingSetting::OSNone) as usize]
                as u64,
        );
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[tph_sett.os_hum.unwrap_or(OversamplingSetting::OSNone) as usize]
                as u64,
        );
        let mut tph_dur = meas_cycles.wrapping_mul(1963u64);
        tph_dur = tph_dur.wrapping_add(477u64.wrapping_mul(4u64));
        tph_dur = tph_dur.wrapping_add(477u64.wrapping_mul(5u64));
        tph_dur = tph_dur.wrapping_add(500u64);
        tph_dur = tph_dur.wrapping_div(1000u64);
        tph_dur = tph_dur.wrapping_add(1u64);
        self.gas_sett.heatr_dur = Some(Duration::from_millis(millis - tph_dur));
    }

    pub fn get_profile_dur(
        &self,
        sensor_settings: &SensorSettings,
    ) -> Result<Duration,<I2C>::Error> {
        let os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        // TODO check if the following unwrap_ors do not change behaviour
        let mut meas_cycles = os_to_meas_cycles[sensor_settings
            .tph_sett
            .os_temp
            .unwrap_or(OversamplingSetting::OSNone)
            as usize] as u32;
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .tph_sett
                .os_pres
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .tph_sett
                .os_hum
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        let mut tph_dur = meas_cycles.wrapping_mul(1963u32);
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
        tph_dur = tph_dur.wrapping_add(500u32);
        tph_dur = tph_dur.wrapping_div(1000u32);
        tph_dur = tph_dur.wrapping_add(1u32);
        let mut duration = Duration::from_millis(tph_dur as u64);
        if sensor_settings.gas_sett.run_gas_measurement {
            duration += sensor_settings.gas_sett.heatr_dur.expect("Heatrdur");
        }
        Ok(duration)
    }

    fn get_calib_data<I2CX>(
        i2c: &mut I2CX,
        dev_id: I2CAddress,
    ) -> Result<CalibData, <I2CX>::Error>
    where
        I2CX: I2c,
    {
        let mut calib: CalibData = Default::default();

        let mut coeff_array: [u8; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN] =
            [0; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN];

        I2CUtil::read_bytes::<I2CX>(
            i2c,
            dev_id.addr(),
            BME680_COEFF_ADDR1,
            &mut coeff_array[0..(BME680_COEFF_ADDR1_LEN - 1)],
        )?;

        I2CUtil::read_bytes::<I2CX>(
            i2c,
            dev_id.addr(),
            BME680_COEFF_ADDR2,
            &mut coeff_array
                [BME680_COEFF_ADDR1_LEN..(BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN - 1)],
        )?;

        calib.par_t1 = ((coeff_array[34usize] as i32) << 8i32 | coeff_array[33usize] as i32) as u16;
        calib.par_t2 = ((coeff_array[2usize] as i32) << 8i32 | coeff_array[1usize] as i32) as i16;
        calib.par_t3 = coeff_array[3usize] as i8;
        calib.par_p1 = ((coeff_array[6usize] as i32) << 8i32 | coeff_array[5usize] as i32) as u16;
        calib.par_p2 = ((coeff_array[8usize] as i32) << 8i32 | coeff_array[7usize] as i32) as i16;
        calib.par_p3 = coeff_array[9usize] as i8;
        calib.par_p4 = ((coeff_array[12usize] as i32) << 8i32 | coeff_array[11usize] as i32) as i16;
        calib.par_p5 = ((coeff_array[14usize] as i32) << 8i32 | coeff_array[13usize] as i32) as i16;
        calib.par_p6 = coeff_array[16usize] as i8;
        calib.par_p7 = coeff_array[15usize] as i8;
        calib.par_p8 = ((coeff_array[20usize] as i32) << 8i32 | coeff_array[19usize] as i32) as i16;
        calib.par_p9 = ((coeff_array[22usize] as i32) << 8i32 | coeff_array[21usize] as i32) as i16;
        calib.par_p10 = coeff_array[23usize];
        calib.par_h1 =
            ((coeff_array[27usize] as i32) << 4i32 | coeff_array[26usize] as i32 & 0xfi32) as u16;
        calib.par_h2 =
            ((coeff_array[25usize] as i32) << 4i32 | coeff_array[26usize] as i32 >> 4i32) as u16;
        calib.par_h3 = coeff_array[28usize] as i8;
        calib.par_h4 = coeff_array[29usize] as i8;
        calib.par_h5 = coeff_array[30usize] as i8;
        calib.par_h6 = coeff_array[31usize];
        calib.par_h7 = coeff_array[32usize] as i8;
        calib.par_gh1 = coeff_array[37usize] as i8;
        calib.par_gh2 =
            ((coeff_array[36usize] as i32) << 8i32 | coeff_array[35usize] as i32) as i16;
        calib.par_gh3 = coeff_array[38usize] as i8;

        calib.res_heat_range =
            (I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RES_HEAT_RANGE_ADDR)?
                & 0x30)
                / 16;

        calib.res_heat_val =
            I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RES_HEAT_VAL_ADDR)? as i8;

        calib.range_sw_err =
            (I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RANGE_SW_ERR_ADDR)?
                & BME680_RSERROR_MSK)
                / 16;

        Ok(calib)
    }

    fn set_gas_config(
        &mut self,
        gas_sett: GasSett,
    ) -> Result<(), <I2C>::Error> {
        if self.power_mode != PowerMode::ForcedMode {
            return Err(Error::DefinePwrMode);
        }

        // TODO check whether unwrap_or changes behaviour
        let reg: [(u8, u8); 2] = [
            (
                BME680_RES_HEAT0_ADDR,
                Calc::calc_heater_res(
                    &self.calib,
                    gas_sett.ambient_temperature,
                    gas_sett.heatr_temp.unwrap_or(0),
                ),
            ),
            (
                BME680_GAS_WAIT0_ADDR,
                Calc::calc_heater_dur(gas_sett.heatr_dur.unwrap_or_else(|| Duration::from_secs(0))),
            ),
        ];

        self.gas_sett.nb_conv = 0;
        self.bme680_set_regs(&reg)
    }

    fn get_gas_config(&mut self) -> Result<GasSett, <I2C>::Error> {
        let heatr_temp = Some(I2CUtil::read_byte(
            &mut self.i2c,
            self.dev_id.addr(),
            BME680_ADDR_SENS_CONF_START,
        )? as u16);

        let heatr_dur_ms = I2CUtil::read_byte(
            &mut self.i2c,
            self.dev_id.addr(),
            BME680_ADDR_GAS_CONF_START,
        )? as u64;

        let gas_sett = GasSett {
            heatr_temp,
            heatr_dur: Some(Duration::from_millis(heatr_dur_ms)),
            ..Default::default()
        };

        Ok(gas_sett)
    }

    /// Retrieve the current sensor informations
    pub fn get_sensor_data(
        &mut self,
        delay: &mut D,
    ) -> Result<(FieldData, FieldDataCondition), <I2C>::Error> {
        let mut buff: [u8; BME680_FIELD_LENGTH] = [0; BME680_FIELD_LENGTH];

        debug!("Buf {:?}, len: {}", buff, buff.len());
        let mut data: FieldData = Default::default();

        const TRIES: u8 = 10;
        for _ in 0..TRIES {
            I2CUtil::read_bytes(
                &mut self.i2c,
                self.dev_id.addr(),
                BME680_FIELD0_ADDR,
                &mut buff,
            )?;

            debug!("Field data read {:?}, len: {}", buff, buff.len());

            data.status = buff[0] & BME680_NEW_DATA_MSK;
            data.gas_index = buff[0] & BME680_GAS_INDEX_MSK;
            data.meas_index = buff[1];

            let adc_pres = (buff[2] as u32).wrapping_mul(4096) | (buff[3] as u32).wrapping_mul(16) | (buff[4] as u32).wrapping_div(16);
            let adc_temp = (buff[5] as u32).wrapping_mul(4096) | (buff[6] as u32).wrapping_mul(16) | (buff[7] as u32).wrapping_div(16);
            let adc_hum = ((buff[8] as u32).wrapping_mul(256) | buff[9] as u32) as u16;
            let adc_gas_res_low = ((buff[13] as u32).wrapping_mul(4) | (buff[14] as u32).wrapping_div(64)) as u16;
            let adc_gas_res_high = ((buff[15] as u32).wrapping_mul(4) | (buff[16] as u32).wrapping_div(64)) as u16;
            let gas_range_l = buff[14] & BME680_GAS_RANGE_MSK;
            let gas_range_h = buff[16] & BME680_GAS_RANGE_MSK;

            // let adc_gas_res = ((buff[13] as u32).wrapping_mul(4) | (buff[14] as u32).wrapping_div(64)) as u16;
            // let gas_range = buff[14] & BME680_GAS_RANGE_MSK;

            if self.varient_id == VarientId::BME688 {
                data.status |= buff[16] & BME680_GASM_VALID_MSK;
                data.status |= buff[16] & BME680_HEAT_STAB_MSK;
            } else {
                data.status |= buff[14] & BME680_GASM_VALID_MSK;
                data.status |= buff[14] & BME680_HEAT_STAB_MSK;
            }


            // I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_RES_HEAT0_ADDR + data.gas_index)?;
            // I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_GAS_WAIT0_ADDR + data.gas_index)?;
            // I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_GAS_IDAC_ADDR + data.gas_index)?;

            if data.status & BME680_NEW_DATA_MSK != 0 {
                let (temp, t_fine) =
                    Calc::calc_temperature(&self.calib, adc_temp, self.tph_sett.temperature_offset);
                debug!(
                    "adc_temp: {} adc_pres: {} adc_hum: {} adc_gas_res_low: {}, adc_gas_res_high: {}, t_fine: {}",
                    adc_temp, adc_pres, adc_hum, adc_gas_res_low, adc_gas_res_high, t_fine
                );
                data.temperature = temp;
                data.pressure = Calc::calc_pressure(&self.calib, t_fine, adc_pres);
                data.humidity = Calc::calc_humidity(&self.calib, t_fine, adc_hum);

                if self.varient_id == VarientId::BME688 {
                    data.gas_resistance = Calc::calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
                } else {
                    data.gas_resistance = Calc::calc_gas_resistance_low(&self.calib, adc_gas_res_low, gas_range_l);
                }

                return Ok((data, FieldDataCondition::NewData));
            }

            delay
                .delay_ms(BME680_POLL_PERIOD_MS as u32);
        }
        Ok((data, FieldDataCondition::Unchanged))
    }
}
