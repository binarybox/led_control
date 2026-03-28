#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Instant;
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Event, Input, InputConfig, Pull},
    rmt::Rmt,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::SmartLedsAdapter;
use esp_println as _;
use module::{
    LedMode,
    constants::{COLOR, LED_OFF, SWITCH_DURATION},
};
use smart_leds::{RGB8, SmartLedsWrite as _};
// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

trait SwitchLed {
    fn led_off(&mut self);
    fn led_on(&mut self, brightness: u8);
}

impl<'a, const BUFFER_SIZE: usize> SwitchLed for SmartLedsAdapter<'a, BUFFER_SIZE, RGB8> {
    fn led_off(&mut self) {
        self.write([LED_OFF; 48]).unwrap();
    }
    fn led_on(&mut self, brightness: u8) {
        self.write(smart_leds::brightness(
            (0..48).map(|idx| if idx % 2 == 0 { LED_OFF } else { COLOR }),
            brightness,
        ))
        .unwrap()
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // would be nicer with lower cpu speed. maybe I will find this out in the future
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initalized");

    let frequency = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, frequency).expect("Failed to initialize RMT0");

    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut kitchen =
        SmartLedsAdapter::new_with_color(rmt.channel0, peripherals.GPIO25, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut dining =
        SmartLedsAdapter::new_with_color(rmt.channel1, peripherals.GPIO26, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut bed_front =
        SmartLedsAdapter::new_with_color(rmt.channel2, peripherals.GPIO12, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut bed_back =
        SmartLedsAdapter::new_with_color(rmt.channel3, peripherals.GPIO5, &mut buffer);

    // Initialize the rotary encoder

    let mut switch = Input::new(
        peripherals.GPIO13,
        InputConfig::default().with_pull(Pull::Down),
    );

    let mut last_offswitch = Instant::MIN;
    let mut mode = LedMode::default();

    loop {
        switch.wait_for(Event::HighLevel).await;

        if last_offswitch.elapsed() < SWITCH_DURATION {
            mode = mode.next();
        }
        info!("switch to mode {} after {}", mode, last_offswitch.elapsed());
        match mode {
            LedMode::AllOn => {
                kitchen.led_on(50);
                dining.led_on(50);
                bed_front.led_on(50);
                bed_back.led_on(50);
            }
            LedMode::OnlyBed => {
                bed_front.led_on(50);
                bed_back.led_on(50);
            }
            LedMode::Front => {
                kitchen.led_on(50);
                dining.led_on(50);
            }
            LedMode::FrontDimmed => {
                kitchen.led_on(10);
                dining.led_on(10);
            }
        }

        switch.wait_for(Event::LowLevel).await;

        info!("switch is low");
        last_offswitch = Instant::now();
        kitchen.led_off();
        dining.led_off();
        bed_front.led_off();
        bed_back.led_off();
    }
}
