#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayUs;
use panic_probe as _;
use rp_pico::entry;

use hal::{gpio, pac, prelude::*, I2C};
use rp_pico::hal;

use core::time::Duration;
use fugit::RateExtU32;

use bme680::{
    Bme680, FieldDataCondition, I2CAddress, IIRFilterSize, OversamplingSetting, PowerMode,
    SettingsBuilder,
};

#[entry]
fn main() -> ! {
    debug!("Start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Wrapper to implement DelayUs for a Cortex M Delay
    struct Delayer(Delay);

    impl DelayUs for Delayer {
        fn delay_us(&mut self, us: u32) {
            self.0.delay_us(us);
        }
    }

    let mut delay = Delayer(Delay::new(core.SYST, clocks.system_clock.freq().to_Hz()));

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init I2C
    let sda_pin = pins.gpio0.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio1.into_mode::<gpio::FunctionI2C>();

    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut dev = Bme680::init(i2c, &mut delay, I2CAddress::Secondary).unwrap();

    let settings = SettingsBuilder::new(&dev)
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_temperature_offset(-2.2)
        .with_run_gas(true)
        .build();

    let profile_dur = dev.get_profile_dur(&settings.0).unwrap();
    info!("Profile duration {:?}", profile_dur);
    info!("Setting sensor settings");
    dev.set_sensor_settings(&mut delay, settings).unwrap();
    info!("Setting forced power modes");

    loop {
        delay.delay_ms(1000);

        match dev.get_sensor_mode() {
            Ok(PowerMode::ForcedMode) => {
                info!("ON");
            }
            Ok(PowerMode::SleepMode) => {
                info!("SLEEP");
            }
            Err(_) => {
                error!("GETTING SENSOR MODE");
            }
        }

        dev.set_sensor_mode(&mut delay, PowerMode::ForcedMode)
            .unwrap();

        match dev.get_sensor_data(&mut delay) {
            Ok((data, state)) => {
                info!("TEMPERATURE {}", data.temperature_celsius());
                info!("HUMIDITY {}", data.humidity_percent());
                info!("PRESSURE {}", data.pressure_hpa());
                info!("GAS RESISTANCE {}", data.gas_resistance_ohm());
                info!("GAS VALID {}", data.gas_valid());
                info!("HEATER STABLE {}", data.heat_stable());
                match state {
                    FieldDataCondition::Unchanged => info!("NO NEW DATA"),
                    FieldDataCondition::NewData => info!("NEW DATA"),
                }
            }
            Err(_) => {
                error!("GETTING SENSOR DATA");
            }
        }
    }
}
