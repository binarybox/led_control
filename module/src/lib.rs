#![no_std]

use defmt::{error, info};
use embassy_time::{Duration, Timer};
use esp_hal::Async;
#[embassy_executor::task]
pub async fn led(i2c: esp_hal::i2c::master::I2c<'static, Async>) {
    let mut aht = ahtx0::ahtx0(i2c);

    let mut delay = embassy_time::Delay;
    info!("driver init");
    let state = aht.soft_reset().await;
    match state {
        Ok(state) => info!("state: {}", state),
        Err(err) => match err {
            ahtx0::Error::Crc => error!("crc error"),
            ahtx0::Error::Timeout => error!("timeout error"),
            ahtx0::Error::I2c(err) => error!("i2c error: {}", err),
        },
    }
    aht.calibrate(&mut delay).await.unwrap();
    info!("calibrated: {}", aht.is_calibrated().await.unwrap());
    Timer::after_millis(100).await;
    info!("driver init done");

    loop {
        match aht.measure(&mut delay).await {
            Ok(measure) => info!(
                "humidity: {} %, temperature {} °C",
                measure.humidity.as_percent(),
                measure.temperature.as_degrees_celsius()
            ),
            Err(err) => match err {
                ahtx0::Error::Crc => error!("crc error"),
                ahtx0::Error::Timeout => error!("timeout error"),
                ahtx0::Error::I2c(err) => error!("i2c error: {}", err),
            },
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}
