#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::net::Ipv4Addr;

use crate::alloc::string::ToString;
use blinksy::color::{FromColor, Hsv, HsvHueRainbow, LedRgb, LedRgbw, RgbChannels};
use blinksy::markers::Dim1d;
use blinksy::pattern::Pattern;
use blinksy::patterns::noise::{Noise1d, NoiseParams};
use blinksy::patterns::rainbow::RainbowParams;
use blinksy::{ControlBuilder, layout::Layout1d, leds::Ws2812};
use blinksy_esp::ClocklessRmtBuilder;
use blinksy_esp::time::elapsed;
use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{self, Input, InputConfig, InputPin, Output, OutputConfig};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::Controller;
use esp_radio::wifi::{AccessPointConfig, WifiDevice};
use leasehund::DhcpServer;
use module::led;
use rotary_encoder_embedded::standard::StandardMode;
use rotary_encoder_embedded::{Direction, RotaryEncoder};
use smoltcp::socket::dhcpv4::{Event, Socket};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

pub struct Dim {
    speed: f32,
}

impl<Layout> Pattern<Dim1d, Layout> for Dim
where
    Layout: Layout1d,
{
    type Params = f32;
    type Color = Hsv<HsvHueRainbow>;

    /// Creates a new Rainbow pattern with the specified parameters.
    fn new(params: Self::Params) -> Self {
        Self { speed: params }
    }

    /// Generates colors for a 1D layout.
    ///
    /// The rainbow pattern creates a smooth transition of hues across the layout,
    /// which shifts over time to create a flowing effect.
    fn tick(&self, time_in_ms: u64) -> impl Iterator<Item = Self::Color> {
        let Self { speed } = self;

        let min_value = 0.05;

        let value = ((((time_in_ms as f32) - 3.0) / (speed - 3.0)) + min_value).min(1.0);

        Layout::points()
            .enumerate()
            .map(move |(index, _)| match time_in_ms {
                0 => Self::Color::new(0.18, 0.9, 0.0),
                1 => Self::Color::new(0.18, 0.9, if index % 8 == 0 { min_value } else { 0.0 }),
                2 => Self::Color::new(0.18, 0.9, if index % 4 == 0 { min_value } else { 0.0 }),
                3 => Self::Color::new(0.18, 0.9, if index % 2 == 0 { min_value } else { 0.0 }),
                _ => Self::Color::new(0.18, 0.9, value),
            })
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.1.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initalized");

    // Define the LED layout (1D strip of 300 pixels)
    blinksy::layout1d!(Layout, 48);

    // Setup the WS2812 driver using RMT.
    let ws2812_driver = {
        // Initialize RMT peripheral (typical base clock 80 MHz).
        let rmt_clk_freq = esp_hal::time::Rate::from_mhz(80);
        let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, rmt_clk_freq).unwrap();
        let rmt_channel = rmt.channel0;

        // Create the driver using the ClocklessRmt builder."]
        blinksy::driver::ClocklessDriver::default()
            .with_led::<Ws2812>()
            .with_writer(
                ClocklessRmtBuilder::default()
                    .with_rmt_buffer_size::<{ Layout::PIXEL_COUNT * 3 * 8 + 1 }>()
                    .with_led::<Ws2812>()
                    .with_channel(rmt_channel)
                    .with_pin(peripherals.GPIO13)
                    .build(),
            )
    };

    let mut pin = Output::new(peripherals.GPIO4, gpio::Level::Low, OutputConfig::default());
    pin.set_low();

    // Build the Blinky controller
    let mut control = ControlBuilder::new_1d()
        .with_layout::<Layout, { Layout::PIXEL_COUNT }>()
        .with_pattern::<Dim>(20.0)
        .with_driver(ws2812_driver)
        .with_frame_buffer_size::<{ Ws2812::frame_buffer_size(Layout::PIXEL_COUNT) }>()
        .build();

    control.set_brightness(1.0); // Set initial brightness (0.0 to 1.0)

    info!("LED initialized!");

    let rotary_dt = Input::new(peripherals.GPIO12, InputConfig::default());
    let rotary_clk = Input::new(peripherals.GPIO26, InputConfig::default());

    // Initialize the rotary encoder
    let mut rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk).into_standard_mode();

    let mut incrementer: u64 = 0;
    let mut switch = Input::new(
        peripherals.GPIO25,
        InputConfig::default().with_pull(gpio::Pull::Down),
    );
    switch.listen(gpio::Event::RisingEdge);

    let mut timestamp = Instant::from_millis(0);
    loop {
        if switch.is_interrupt_set() {
            timestamp = Instant::now();
            switch.clear_interrupt();
            switch.listen(gpio::Event::RisingEdge);
        }
        if timestamp.elapsed() <= Duration::from_secs(3) {
            match rotary_encoder.update() {
                Direction::Clockwise => {
                    incrementer = incrementer.wrapping_add(1).min(20);
                    control.tick(incrementer).unwrap();
                    timestamp = Instant::now();
                }
                Direction::Anticlockwise => {
                    incrementer = incrementer.saturating_sub(1);
                    control.tick(incrementer).unwrap();
                    timestamp = Instant::now();
                }

                Direction::None => {
                    // Do nothing
                }
            }
        }
    }

    // let i2c =
    //     esp_hal::i2c::master::I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
    //         .unwrap()
    //         .with_scl(peripherals.GPIO22)
    //         .with_sda(peripherals.GPIO21)
    //         .into_async();

    // spawner.spawn(led(i2c)).unwrap();

    // static RADIO_INIT: StaticCell<Controller> = StaticCell::new();
    // let radio_init =
    //     RADIO_INIT.init(esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller"));

    // let (mut wifi_controller, interfaces) =
    //     esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
    //         .expect("Failed to initialize Wi-Fi controller");

    // wifi_controller
    //     .set_config(&esp_radio::wifi::ModeConfig::AccessPoint(
    //         AccessPointConfig::default().with_ssid("esp-wifi".to_string()),
    //     ))
    //     .unwrap();

    // wifi_controller.start_async().await.unwrap();

    // static RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
    // let config = embassy_net::Config::dhcpv4(Default::default());
    // static STACK_RUNNER: StaticCell<(Stack, Runner<'static, WifiDevice<'static>>)> =
    //     StaticCell::new();

    // let (stack, runner) = STACK_RUNNER.init(embassy_net::new(
    //     interfaces.ap,
    //     config,
    //     RESOURCES.init(embassy_net::StackResources::new()),
    //     1823,
    // ));

    // spawner.spawn(network_runner(runner)).unwrap();

    // spawner.spawn(dhcp_server(stack)).unwrap();

    loop {
        let elapsed_in_ms = elapsed().as_millis();
        control.tick(elapsed_in_ms).unwrap();
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn dhcp_server(stack: &'static mut Stack<'static>) {
    let mut dhcp_server: DhcpServer<32, 4> = DhcpServer::new_with_dns(
        Ipv4Addr::new(192, 168, 1, 1),   // Server IP
        Ipv4Addr::new(255, 255, 255, 0), // Subnet mask
        Ipv4Addr::new(192, 168, 1, 1),   // Router/Gateway
        Ipv4Addr::new(8, 8, 8, 8),       // DNS server
        Ipv4Addr::new(192, 168, 1, 100), // IP pool start
        Ipv4Addr::new(192, 168, 1, 200), // IP pool end
    );
    dhcp_server.run(*stack).await;
}

#[embassy_executor::task]
async fn network_runner(runner: &'static mut Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}
