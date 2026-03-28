#![no_std]

pub mod constants;

#[derive(Debug, defmt::Format, Default)]
pub enum LedMode {
    #[default]
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
