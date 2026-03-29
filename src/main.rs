#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Event, Input, InputConfig, Pull},
    rmt::{PulseCode, Rmt},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::SmartLedsAdapterAsync;
use esp_println as _;
use module::{
    LedMode,
    constants::{COLOR, LED_OFF, SWITCH_DURATION},
};
use smart_leds::{RGB8, SmartLedsWriteAsync};
// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

trait SwitchLed {
    async fn led_off(&mut self);
    async fn led_on(&mut self, brightness: u8);
}

impl<'a, const BUFFER_SIZE: usize> SwitchLed for SmartLedsAdapterAsync<'a, BUFFER_SIZE, RGB8> {
    async fn led_off(&mut self) {
        self.write([LED_OFF; 48])
            .await
            .inspect_err(|e| defmt::error!("led off got error {:?}", e))
            .unwrap();
    }
    async fn led_on(&mut self, brightness: u8) {
        self.write(smart_leds::brightness(
            (0..48).map(|idx| if idx % 2 == 0 { LED_OFF } else { COLOR }),
            brightness,
        ))
        .await
        .inspect_err(|e| defmt::error!("led on got error {:?}", e))
        .unwrap()
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // TODO would be nicer with lower cpu speed. maybe I will find this out in the future
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initalized");

    let frequency = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, frequency)
        .expect("Failed to initialize RMT0")
        .into_async();

    // initalizing the leds
    let mut buffer = [PulseCode::end_marker(); esp_hal_smartled::buffer_size_async(48)];
    let mut kitchen =
        SmartLedsAdapterAsync::new_with_color(rmt.channel0, peripherals.GPIO25, &mut buffer);
    let mut buffer = [PulseCode::end_marker(); esp_hal_smartled::buffer_size_async(48)];
    let mut dining =
        SmartLedsAdapterAsync::new_with_color(rmt.channel1, peripherals.GPIO26, &mut buffer);
    let mut buffer = [PulseCode::end_marker(); esp_hal_smartled::buffer_size_async(48)];
    let mut bed_front =
        SmartLedsAdapterAsync::new_with_color(rmt.channel2, peripherals.GPIO12, &mut buffer);
    let mut buffer = [PulseCode::end_marker(); esp_hal_smartled::buffer_size_async(48)];
    let mut bed_back =
        SmartLedsAdapterAsync::new_with_color(rmt.channel3, peripherals.GPIO5, &mut buffer);

    info!("LEDS initalized");

    // initalizing the switch
    let mut switch = Input::new(
        peripherals.GPIO13,
        InputConfig::default().with_pull(Pull::Down),
    );

    info!("Switch initalized");

    let mut last_offswitch = Instant::MIN;
    let mut mode = LedMode::default();

    loop {
        switch.wait_for(Event::HighLevel).await;

        // this prevents flickering. Since the hardware setup is not so robust, we do need this in the software
        embassy_time::Timer::after(Duration::from_millis(100)).await;
        if switch.is_low() {
            warn!("switch is flickering");
            continue;
        }

        if last_offswitch.elapsed() < SWITCH_DURATION {
            mode = mode.next();
        }
        info!("switch to mode {} after {}", mode, last_offswitch.elapsed());
        match mode {
            LedMode::AllOn => {
                kitchen.led_on(50).await;
                dining.led_on(50).await;
                bed_front.led_on(50).await;
                bed_back.led_on(50).await;
            }
            LedMode::OnlyBed => {
                bed_front.led_on(50).await;
                bed_back.led_on(50).await;
            }
            LedMode::Front => {
                kitchen.led_on(50).await;
                dining.led_on(50).await;
            }
            LedMode::FrontDimmed => {
                kitchen.led_on(10).await;
                dining.led_on(10).await;
            }
        }

        switch.wait_for(Event::LowLevel).await;

        info!("switch is low");
        last_offswitch = Instant::now();
        kitchen.led_off().await;
        dining.led_off().await;
        bed_front.led_off().await;
        bed_back.led_off().await;
    }
}
