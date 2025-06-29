use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::mem::size_of;
use core::ptr;
// use crate::app::sbg_flush;
// use crate::app::sbg_handle_data;
// use crate::app::sbg_sd_task as sbg_sd;
// use crate::app::sbg_write_data;
use super::RTC;
use super::UART_CHANNEL;
use crate::SBG_CHANNEL;
use chrono::NaiveDateTime;
use core::mem::MaybeUninit;
use embassy_stm32::mode;
use embassy_stm32::usart::UartTx;
use embedded_alloc::Heap;
use heapless::Vec;
use sbg_rs::sbg;
use sbg_rs::sbg::{CallbackData, SBG, SBG_BUFFER_SIZE};
// use stm32h7xx_hal::dma::dma::StreamX;
// use stm32h7xx_hal::dma::{
//     dma::{DmaConfig, StreamsTuple},
//     PeripheralToMemory, Transfer,
// };
// use stm32h7xx_hal::pac::UART4;
// use stm32h7xx_hal::serial::{Rx, Tx};
use rtic::Mutex;

// must have this link section.
#[link_section = ".axisram.buffers"]
pub static mut SBG_BUFFER: MaybeUninit<[u8; SBG_BUFFER_SIZE]> = MaybeUninit::uninit();

#[global_allocator]
static HEAP: Heap = Heap::empty();

pub struct SBGManager {
    pub sbg_device: SBG,
    sbg_tx: UartTx<'static, mode::Async>,
}

impl SBGManager {
    pub fn new(sbg_tx: UartTx<'static, mode::Async>) -> Self {
        /* Initialize the Heap */
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            // TODO: Could add a link section here to memory.
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        // let (sbg_tx, sbg_rx) = serial.split();

        unsafe {
            // Convert an uninitialised array into an array of uninitialised
            let buf: &mut [core::mem::MaybeUninit<u8>; SBG_BUFFER_SIZE] =
                &mut *(core::ptr::addr_of_mut!(SBG_BUFFER) as *mut _);
            buf.iter_mut().for_each(|x| x.as_mut_ptr().write(0));
        }

        // let config = DmaConfig::default()
        //     .memory_increment(true)
        //     .transfer_complete_interrupt(true);
        // let mut transfer: Transfer<
        //     StreamX<stm32h7xx_hal::pac::DMA1, 1>,
        //     Rx<stm32h7xx_hal::pac::UART4>,
        //     PeripheralToMemory,
        //     &mut [u8; SBG_BUFFER_SIZE],
        //     stm32h7xx_hal::dma::DBTransfer,
        // > = Transfer::init(
        //     stream_tuple.1,
        //     sbg_rx,
        //     unsafe { SBG_BUFFER.assume_init_mut() }, // Uninitialised memory
        //     None,
        //     config,
        // );

        // info!("Starting transfer");

        // transfer.start(|serial| {
        //     serial.enable_dma_rx();
        // });
        // info!("Transfer started");

        let sbg: sbg::SBG = sbg::SBG::new(
            |data| {
                sbg_handle_data(data);
            },
            |data| {

                // sbg_write_data::spawn(data).ok();
            },
            || sbg_get_time(),
            || {

                // sbg_flush::spawn().ok();
            },
        );

        SBGManager {
            sbg_device: sbg,
            sbg_tx,
        }
    }
}

pub async fn sbg_flush() {}

pub async fn sbg_write_data(data: Vec<u8, SBG_BUFFER_SIZE>) {}

pub fn sbg_get_time_millis_i64() -> i64 {
    RTC.lock(|cell| {
        // First, get a mutable reference to the contents of the RefCell.
        // This will panic at runtime if another part of the code already has a mutable borrow.
        let mut rtc_uninit = cell.borrow_mut();

        // Now, `rtc_uninit` is a `&mut MaybeUninit<Rtc>`, which is what `assume_init_mut` needs.
        let rtc = unsafe { rtc_uninit.assume_init_mut() };

        // The rest of your logic is correct.
        rtc.now()
            .ok()
            .map(|embassy_dt| {
                let chrono_naive_dt: NaiveDateTime = embassy_dt.into();
                chrono_naive_dt.and_utc().timestamp_millis()
            })
            .unwrap_or(0)
    })
}

pub fn sbg_get_time() -> u32 {
    // We get the full i64 timestamp first.
    let timestamp_ms = sbg_get_time_millis_i64();

    // Convert to u32. This is a truncating cast.
    // For positive timestamps, it's equivalent to `timestamp_ms % (u32::MAX as i64 + 1)`.
    timestamp_ms as u32
}

// pub fn sbg_get_time() -> u32 {
//     cortex_m::interrupt::free(|cs| {

//         let mut rc = RTC.borrow(cs).borrow_mut();
//         let rtc = rc.as_mut().unwrap();
//         rtc.date_time()
//             .unwrap_or(NaiveDateTime::new(
//                 NaiveDate::from_ymd_opt(2024, 1, 1).unwrap(),
//                 NaiveTime::from_hms_milli_opt(0, 0, 0, 0).unwrap(),
//             ))
//             .and_utc()
//             .timestamp_subsec_millis()
//     })
// }

/// Publishes data to the SBG channel.
pub fn sbg_handle_data(data: CallbackData) {
    match data {
        CallbackData::Air(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::Air(x)),
        }),
        CallbackData::EkfNav(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::EkfNav(x)),
        }),
        CallbackData::EkfQuat(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::EkfQuat(x)),
        }),
        CallbackData::GpsPos(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::GpsPos(x)),
        }),
        CallbackData::GpsVel(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::GpsVel(x)),
        }),
        CallbackData::Imu(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::Imu(x)),
        }),
        CallbackData::UtcTime(x) => SBG_CHANNEL.send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::UtcTime(x)),
        }),
    };
    // cx.shared.data_manager.lock(|manager| match data {
    //     CallbackData::UtcTime(x) => manager.utc_time = Some(x),
    //     CallbackData::Air(x) => manager.air = Some(x),
    //     CallbackData::EkfQuat(x) => manager.ekf_quat = Some(x),
    //     CallbackData::EkfNav(x) => manager.ekf_nav = Some(x),
    //     CallbackData::Imu(x) => manager.imu = Some(x),
    //     CallbackData::GpsVel(x) => manager.gps_vel = Some(x),
    //     CallbackData::GpsPos(x) => manager.gps_pos = Some(x),
    // });
}

pub async fn sbg_sd_task(data: [u8; SBG_BUFFER_SIZE]) {

    // cx.shared.sd_manager.lock(|manager| {
    //     if let Some(mut file) = manager.file.take() {
    //         cx.shared.em.run(|| {
    //             manager.write(&mut file, &data)?;
    //             Ok(())
    //         });
    //         manager.file = Some(file); // give the file back after use
    //     } else if let Ok(mut file) = manager.open_file("lc24.txt") {
    //         cx.shared.em.run(|| {
    //             manager.write(&mut file, &data)?;
    //             Ok(())
    //         });
    //         manager.file = Some(file);
    //     }
    // });
}
/**
 * Recieves data from the UART channel and sends it to the sbg library for processing.
 */
#[embassy_executor::task]
pub async fn sbg_dma() {
    loop {
        let data = UART_CHANNEL.receive().await;
    }

    // cx.shared.sbg_manager.lock(|sbg| {
    //     match &mut sbg.xfer {
    //         Some(xfer) => {
    //             if xfer.get_transfer_complete_flag() {
    //                 let data = unsafe { SBG_BUFFER.assume_init_read() }.clone();
    //                 xfer.next_transfer(
    //                     unsafe { (*core::ptr::addr_of_mut!(SBG_BUFFER)).assume_init_mut() }, // Uninitialised memory
    //                 );
    //                 // info!("{}", data);
    //                 xfer.clear_transfer_complete_interrupt();
    //                 sbg.sbg_device.read_data(&data);
    //                 crate::app::sbg_sd_task::spawn(data).ok();
    //             }
    //         }
    //         None => {
    //             // it should be impossible to reach here.
    //             info!("None");
    //         }
    //     }
    // });
}

/// Stored right before an allocation. Stores information that is needed to deallocate memory.
#[derive(Copy, Clone)]
struct AllocInfo {
    layout: Layout,
    ptr: *mut u8,
}

/// Custom malloc for the SBG library. This uses the HEAP object initialized at the start of the
/// [`SBGManager`]. The [`Layout`] of the allocation is stored right before the returned pointed,
/// which makes it possible to implement [`free`] without any other data structures.
#[no_mangle]
pub extern "C" fn malloc(size: usize) -> *mut c_void {
    if size == 0 {
        return ptr::null_mut();
    }

    // Get a layout for both the requested size
    let header_layout = Layout::new::<AllocInfo>();
    let requested_layout = Layout::from_size_align(size, 8).unwrap();
    let (layout, offset) = header_layout.extend(requested_layout).unwrap();

    // Ask the allocator for memory
    let orig_ptr = unsafe { HEAP.alloc(layout) };
    if orig_ptr.is_null() {
        return orig_ptr as *mut c_void;
    }

    // Compute the pointer that we will return
    let result_ptr = unsafe { orig_ptr.add(offset) };

    // Store the allocation information right before the returned pointer
    let info_ptr = unsafe { result_ptr.sub(size_of::<AllocInfo>()) as *mut AllocInfo };
    unsafe {
        info_ptr.write_unaligned(AllocInfo {
            layout,
            ptr: orig_ptr,
        });
    }

    result_ptr as *mut c_void
}

/// Custom free implementation for the SBG library. This uses the stored allocation information
/// right before the pointer to free up the resources.
///
/// SAFETY: The value passed to ptr must have been obtained from a previous call to [`malloc`].
#[no_mangle]
pub unsafe extern "C" fn free(ptr: *mut c_void) {
    assert!(!ptr.is_null());

    let info_ptr = unsafe { ptr.sub(size_of::<AllocInfo>()) as *const AllocInfo };
    let info = unsafe { info_ptr.read_unaligned() };
    unsafe {
        HEAP.dealloc(info.ptr, info.layout);
    }
}
