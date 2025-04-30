#![no_std]
#![no_main]

use common_arm as _;
use cortex_m_rt::entry;
use defmt::info;
use panic_probe as _;
use stm32h7xx_hal::{adc, delay::Delay, pac, prelude::*, rcc::rec::AdcClkSel};

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[entry]
fn main() -> ! {
    let _cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    info!("Power enabled");
    // RCC
    let mut rcc = dp.RCC.constrain();
    let reset = rcc.get_reset_reason();

    info!("Reset reason: {:?}", reset);

    let mut ccdr = rcc
        .use_hse(48.MHz()) // check the clock hardware
        .sys_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);
    info!("RCC configured");

    // Configure ADC
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);
    let mut delay = Delay::new(_cp.SYST, ccdr.clocks);
    let mut adc1 = adc::Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks).enable();
    adc1.set_resolution(adc::Resolution::SixteenBit);
    
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    
    let mut main_a_arm    = gpiod.pd6.into_push_pull_output();
    let mut main_a_fire   = gpiod.pd5.into_push_pull_output();
    let mut main_a_sense  = gpioa.pa2.into_analog();

    let mut main_b_arm    = gpiod.pd14.into_push_pull_output();
    let mut main_b_fire   = gpiod.pd13.into_push_pull_output();
    let mut main_b_sense  = gpiob.pb0.into_analog();

    let mut drogue_a_arm  = gpioc.pc11.into_push_pull_output();
    let mut drogue_a_fire = gpioc.pc12.into_push_pull_output();
    let mut drogue_a_sense = gpioa.pa3.into_analog();

    let mut drogue_b_arm  = gpiod.pd2.into_push_pull_output();
    let mut drogue_b_fire = gpiod.pd1.into_push_pull_output();
    let mut drogue_b_sense = gpioc.pc5.into_analog();

    cortex_m::asm::delay(1_000_000);

    main_a_arm.set_low();
    main_a_fire.set_low();
    main_b_arm.set_low();
    main_b_fire.set_low();
    drogue_a_arm.set_low();
    drogue_a_fire.set_low();
    drogue_b_arm.set_low();
    drogue_b_fire.set_low();

    cortex_m::asm::delay(1_000);

    main_a_arm.set_high();
    main_b_arm.set_high();
    drogue_a_arm.set_high();
    drogue_b_arm.set_high();
    
    loop {
        const VREF: f32 = 3.0; // In volts
        let res: f32 = adc1.slope() as f32;

        let main_a_sense: u32 = adc1.read(&mut main_a_sense).unwrap();
        let main_b_sense: u32 = adc1.read(&mut main_b_sense).unwrap();
        let drogue_a_sense: u32 = adc1.read(&mut drogue_a_sense).unwrap();
        let drogue_b_sense: u32 = adc1.read(&mut drogue_b_sense).unwrap();
        
        let main_a_data: f32 = main_a_sense as f32 * (VREF / res);
        let main_b_data: f32 = main_b_sense as f32 * (VREF / res);
        let drogue_a_data: f32 = drogue_a_sense as f32 * (VREF / res);
        let drogue_b_data: f32 = drogue_b_sense as f32 * (VREF / res);

        info!("Main A: {:?}", main_a_data);
        info!("Main B: {:?}", main_b_data);
        info!("Drogue A: {:?}", drogue_a_data);
        info!("Drogue B: {:?}", drogue_b_data);
        info!("\n");

        cortex_m::asm::delay(100_000_000);
    }
}
