use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::mem::size_of;
use core::ptr;
// use crate::app::sbg_flush;
// use crate::app::sbg_handle_data;
// use crate::app::sbg_sd_task as sbg_sd;
// use crate::app::sbg_write_data;
use super::sbg_manager;
use crate::resources::{BUFFER_CHANNEL, PRESSURE_CHANNEL, RADIO_CHANNEL, RTC, SBG_CHANNEL, SD_CHANNEL};
use crate::HEAP;
use chrono::NaiveDateTime;
use defmt::info;
use embassy_stm32::mode;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use heapless::Vec;
use messages_prost::prost::Message;
use sbg_rs::sbg;
use sbg_rs::sbg::{CallbackData, SBG, SBG_BUFFER_SIZE};
// use stm32h7xx_hal::dma::dma::StreamX;
// use stm32h7xx_hal::dma::{
//     dma::{DmaConfig, StreamsTuple},
//     PeripheralToMemory, Transfer,
// };
// use stm32h7xx_hal::pac::UART4;
// use stm32h7xx_hal::serial::{Rx, Tx};

#[embassy_executor::task]
pub async fn uart_dma_reader_task(mut rx: RingBufferedUartRx<'static>) {
    info!("DMA reader task spawned.");
    loop {
        let mut buf: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                let _ = BUFFER_CHANNEL.send(buf).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn sbg_parser_task(tx: UartTx<'static, mode::Async>) {
    let mut sbg = sbg_manager::SBGManager::new(tx);
    loop {
        let full_buffer = BUFFER_CHANNEL.receive().await;
        sbg.sbg_device.read_data(&full_buffer.try_into().unwrap());
    }
}

#[embassy_executor::task]
pub async fn sbg_receiver_task() {
    loop {
        let data = SBG_CHANNEL.receive().await;
        match data.data {
            Some(x) => {
                let mut buf: [u8; 255] = [0; 255];
                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::Sbg(data)),
                };
                msg.encode_length_delimited(&mut buf.as_mut())
                    .expect("Failed to encode SBG GPS Position");
                RADIO_CHANNEL.send(buf.clone()).await;

                SD_CHANNEL.send(("sbg.txt", buf)).await; 
            }
            None => {
                info!("No SBG data received");
            }
        }
    }
}

pub struct SBGManager {
    pub sbg_device: SBG,
    sbg_tx: UartTx<'static, mode::Async>,
}

impl SBGManager {
    pub fn new(sbg_tx: UartTx<'static, mode::Async>) -> Self {
        let sbg: sbg::SBG = sbg::SBG::new(
            |data| {
                sbg_handle_data(data);
            },
            |data| {
                // TODO: implement later
                // sbg_write_data::spawn(data).ok();
            },
            || sbg_get_time(),
            || {
                // TODO: implement later
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
        if let Some(rtc) = cell.borrow_mut().as_mut() {
            rtc.now()
                .ok()
                .map(|embassy_dt| {
                    let chrono_naive_dt: NaiveDateTime = embassy_dt.into();
                    chrono_naive_dt.and_utc().timestamp_millis()
                })
                .unwrap_or(0)
        } else {
            // Handle the case where RTC is not initialized
            0
        }
    })
}

pub fn sbg_get_time() -> u32 {
    // // We get the full i64 timestamp first.
    // let timestamp_ms = sbg_get_time_millis_i64();

    // // Convert to u32. This is a truncating cast.
    // // For positive timestamps, it's equivalent to `timestamp_ms % (u32::MAX as i64 + 1)`.
    // timestamp_ms as u32
    501
}

/// Publishes data to the SBG channel.
pub fn sbg_handle_data(data: CallbackData) {
    match data {
        CallbackData::Air(x) => {
            PRESSURE_CHANNEL.try_send((
                x.data.unwrap().altitude,
                x.data.unwrap().air_temperature,
                0,
                embassy_time::Instant::now(),
            ));
            SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
                data: Some(messages_prost::sensor::sbg::sbg_data::Data::Air(x)),
            })
        }
        CallbackData::EkfNav(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::EkfNav(x)),
        }),
        CallbackData::EkfQuat(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::EkfQuat(x)),
        }),
        CallbackData::GpsPos(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::GpsPos(x)),
        }),
        CallbackData::GpsVel(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::GpsVel(x)),
        }),
        CallbackData::Imu(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::Imu(x)),
        }),
        CallbackData::UtcTime(x) => SBG_CHANNEL.try_send(messages_prost::sensor::sbg::SbgData {
            data: Some(messages_prost::sensor::sbg::sbg_data::Data::UtcTime(x)),
        }),
    };
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
    // loop {
    //     let data = UART_CHANNEL.receive().await;
    // }

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
