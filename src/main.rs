#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
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
use smart_leds::{RGB8, SmartLedsWrite as _, brightness};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const COLOR: RGB8 = RGB8::new(0xff, 0xf0, 0x4d);
const LED_OFF: RGB8 = RGB8::new(0, 0, 0);

const SWITCH_DURATION: Duration = Duration::from_millis(1000);

#[derive(Debug, defmt::Format)]
enum LedMode {
    AllOn,
    OnlyBed,
    Front,
    FrontDimmed,
}

impl LedMode {
    pub fn next(self: Self) -> LedMode {
        match self {
            LedMode::AllOn => LedMode::OnlyBed,
            LedMode::OnlyBed => LedMode::Front,
            LedMode::Front => LedMode::FrontDimmed,
            LedMode::FrontDimmed => LedMode::AllOn,
        }
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
    let mut kitchen = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO25, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut dining = SmartLedsAdapter::new(rmt.channel1, peripherals.GPIO26, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut bed_front = SmartLedsAdapter::new(rmt.channel2, peripherals.GPIO12, &mut buffer);
    let mut buffer = esp_hal_smartled::smart_led_buffer!(48);
    let mut bed_back = SmartLedsAdapter::new(rmt.channel3, peripherals.GPIO5, &mut buffer);

    // Initialize the rotary encoder

    let mut switch = Input::new(
        peripherals.GPIO13,
        InputConfig::default().with_pull(Pull::Down),
    );

    let mut last_offswitch = Instant::MIN;
    let mut mode = LedMode::AllOn;

    loop {
        switch.wait_for(Event::HighLevel).await;

        if last_offswitch.elapsed() < SWITCH_DURATION {
            mode = mode.next();
        }
        info!("switch to mode {} after {}", mode, last_offswitch.elapsed());
        let leds = (0..48).map(|idx| if idx % 2 == 0 { LED_OFF } else { COLOR });
        match mode {
            LedMode::AllOn => {
                kitchen.write(brightness(leds.clone(), 50)).unwrap();
                dining.write(brightness(leds.clone(), 50)).unwrap();
                bed_front.write(leds.clone().into_iter()).unwrap();
                bed_back.write(brightness(leds.clone(), 50)).unwrap();
            }
            LedMode::OnlyBed => {
                kitchen.write([LED_OFF; 48].into_iter()).unwrap();
                dining.write([LED_OFF; 48].into_iter()).unwrap();
                bed_front.write(leds.clone()).unwrap();
                bed_back.write(brightness(leds.clone(), 50)).unwrap();
            }
            LedMode::Front => {
                kitchen
                    .write(brightness(leds.clone().into_iter(), 50))
                    .unwrap();
                dining
                    .write(brightness(leds.clone().into_iter(), 50))
                    .unwrap();
                bed_front.write([LED_OFF; 48].into_iter()).unwrap();
                bed_back.write([LED_OFF; 48].into_iter()).unwrap();
            }
            LedMode::FrontDimmed => {
                kitchen
                    .write(brightness(leds.clone().into_iter(), 10))
                    .unwrap();
                dining
                    .write(brightness(leds.clone().into_iter(), 10))
                    .unwrap();
                bed_front.write([LED_OFF; 48].into_iter()).unwrap();
                bed_back.write([LED_OFF; 48].into_iter()).unwrap();
            }
        }

        switch.wait_for(Event::LowLevel).await;

        info!("switch is low");
        last_offswitch = Instant::now();
        kitchen
            .write(brightness([LED_OFF; 48].into_iter(), 0))
            .unwrap();
        dining
            .write(brightness([LED_OFF; 48].into_iter(), 0))
            .unwrap();
        bed_front
            .write(brightness([LED_OFF; 48].into_iter(), 0))
            .unwrap();
        bed_back
            .write(brightness([LED_OFF; 48].into_iter(), 0))
            .unwrap();
    }
}
