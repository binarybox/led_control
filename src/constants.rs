use embassy_time::Duration;
use smart_leds::RGB8;

pub const COLOR: RGB8 = RGB8::new(0xff, 0xf0, 0x4d);
pub const LED_OFF: RGB8 = RGB8::new(0, 0, 0);

pub const SWITCH_DURATION: Duration = Duration::from_millis(1000);
