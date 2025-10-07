use std::{
    alloc,
    fmt::Debug,
    mem::{self, ManuallyDrop},
    pin::Pin,
    ptr::{addr_of_mut, null_mut},
    slice,
    time::Instant,
};

use log::trace;
use rustix::io::Errno;

use crate::{
    descriptors::TransferType,
    platform::linux_usbfs::usbfs::IsoPacketDesc,
    transfer::{
        internal::Pending, Allocator, Buffer, Completion, ControlIn, ControlOut, Direction,
        TransferError, SETUP_PACKET_SIZE,
    },
};

use super::{
    errno_to_transfer_error,
    usbfs::{
        Urb, USBDEVFS_URB_TYPE_BULK, USBDEVFS_URB_TYPE_CONTROL, USBDEVFS_URB_TYPE_INTERRUPT,
        USBDEVFS_URB_TYPE_ISO,
    },
};

/// Linux-specific transfer state.
///
/// This logically contains a `Vec` with urb.buffer and capacity.
/// It also owns the `urb` allocation itself, which is stored out-of-line
/// to enable isochronous transfers to allocate the variable-length
/// `iso_packet_desc` array.
pub struct TransferData {
    urb: *mut Urb,
    ep_type: TransferType,
    capacity: u32,
    allocator: Allocator,
    pub(crate) deadline: Option<Instant>,
    urb_iso: *mut IsoPacketDesc,
}

impl Debug for TransferData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut f = f.debug_struct("TransferData");
        f.field("urb_ptr", &self.urb);
        f.field("urb", unsafe { &*self.urb });
        f.field("ep_type", &self.ep_type);
        f.field("capacity", &self.capacity);
        f.field("allocator", &self.allocator);
        f.field("deadline", &self.deadline);
        f.field("urb_iso_ptr", &self.urb_iso);
        if self.ep_type == TransferType::Isochronous && !self.urb_iso.is_null() {
            f.field("urb_iso", unsafe {
                &slice::from_raw_parts(
                    self.urb_iso,
                    self.urb().number_of_packets_or_stream_id as usize,
                )
            });
        }

        f.finish()
    }
}

unsafe impl Send for TransferData {}
unsafe impl Sync for TransferData {}

impl TransferData {
    pub(super) fn new(endpoint: u8, ep_type: TransferType) -> TransferData {
        let transfer_type = ep_type;
        let ep_type = match ep_type {
            TransferType::Control => USBDEVFS_URB_TYPE_CONTROL,
            TransferType::Interrupt => USBDEVFS_URB_TYPE_INTERRUPT,
            TransferType::Bulk => USBDEVFS_URB_TYPE_BULK,
            TransferType::Isochronous => USBDEVFS_URB_TYPE_ISO,
        };

        let mut empty = ManuallyDrop::new(Vec::new());

        TransferData {
            urb: Box::into_raw(Box::new(Urb {
                ep_type,
                endpoint,
                status: 0,
                flags: 0,
                buffer: empty.as_mut_ptr(),
                buffer_length: 0,
                actual_length: 0,
                start_frame: 0,
                number_of_packets_or_stream_id: 0,
                error_count: 0,
                signr: 0,
                usercontext: null_mut(),
                // iso_frame_desc: null_mut(),
            })),
            ep_type: transfer_type,
            capacity: 0,
            allocator: Allocator::Default,
            deadline: None,
            urb_iso: null_mut(),
        }
    }

    pub(super) fn new_control_out(data: ControlOut) -> TransferData {
        let mut t = TransferData::new(0x00, TransferType::Control);
        let mut buffer = Buffer::new(SETUP_PACKET_SIZE.checked_add(data.data.len()).unwrap());
        buffer.extend_from_slice(&data.setup_packet());
        buffer.extend_from_slice(data.data);
        t.set_buffer(buffer);
        t
    }

    pub(super) fn new_control_in(data: ControlIn) -> TransferData {
        let mut t = TransferData::new(0x80, TransferType::Control);
        let mut buffer = Buffer::new(SETUP_PACKET_SIZE.checked_add(data.length as usize).unwrap());
        buffer.extend_from_slice(&data.setup_packet());
        t.set_buffer(buffer);
        t
    }

    pub fn set_buffer(&mut self, buf: Buffer) {
        // debug_assert_eq!(self.ep_type, TransferType::Isochronous);
        debug_assert!(self.capacity == 0);
        let buf = ManuallyDrop::new(buf);
        self.capacity = buf.capacity;
        self.urb_mut().buffer = buf.ptr;
        self.urb_mut().actual_length = 0;
        self.urb_mut().buffer_length = match Direction::from_address(self.urb().endpoint) {
            Direction::Out => buf.len as i32,
            Direction::In => buf.requested_len as i32,
        };
        self.allocator = buf.allocator;
    }

    pub fn set_iso_buffer(&mut self, buf: Buffer, iso_packet_amount: usize, iso_packet_size: u32) {
        trace!("Buffer for iso submit: {buf:#?}");

        debug_assert_eq!(self.ep_type, TransferType::Isochronous);
        debug_assert_ne!(iso_packet_size, 0, "Buffer capacity: {}", buf.capacity);

        let alloc_size = size_of::<Urb>() + iso_packet_amount * size_of::<IsoPacketDesc>();
        trace!(
            "Alloc size: {alloc_size}; Urb size: {}; Urb align: {}; Iso Desc size: {}",
            size_of::<Urb>(),
            mem::align_of::<Urb>(),
            size_of::<IsoPacketDesc>()
        );
        let layout = alloc::Layout::from_size_align(alloc_size, mem::align_of::<Urb>()).unwrap();
        let ptr = unsafe { alloc::alloc_zeroed(layout) } as *mut Urb;
        trace!("Allocated by: {ptr:?}");

        let old_urb = self.urb().clone();
        if !self.urb_iso.is_null() {
            let for_drop = unsafe { Box::from_raw(self.urb) };
        } else {
            let num_packets = self.urb().number_of_packets_or_stream_id as usize;
            let alloc_size = size_of::<Urb>() + num_packets * size_of::<IsoPacketDesc>();
            let layout =
                alloc::Layout::from_size_align(alloc_size, mem::align_of::<Urb>()).unwrap();
            unsafe { alloc::dealloc(self.urb as *mut _, layout) };

            self.urb_iso = null_mut();
        };
        self.urb = ptr;
        self.urb_iso = unsafe { ptr.add(1) as *mut _ };
        trace!("Urb iso: {:?}", self.urb_iso);

        self.urb_mut().endpoint = old_urb.endpoint;
        self.urb_mut().ep_type = old_urb.ep_type;

        self.urb_mut().number_of_packets_or_stream_id = iso_packet_amount as u32;
        self.set_buffer(buf);

        const USBFS_URB_ISO_ASAP: u32 = 0x02;
        const USER_CONTEXT: &str = "Some string";
        self.urb_mut().flags = USBFS_URB_ISO_ASAP;
        self.urb_mut().usercontext = USER_CONTEXT.as_ptr() as *mut _;

        let iso_packets = unsafe { slice::from_raw_parts_mut(self.urb_iso, iso_packet_amount) };
        for i in 0..iso_packet_amount {
            iso_packets[i].length = iso_packet_size as u32;
            iso_packets[i].actual_length = 0;
            iso_packets[i].status = 0;
        }
    }

    pub fn end_iso(&self) -> Option<bool> {
        if self.ep_type == TransferType::Isochronous {
            let urb = self.urb();
            let packets = unsafe {
                slice::from_raw_parts(self.urb_iso, urb.number_of_packets_or_stream_id as usize)
            };
            Some(packets.iter().all(|p| p.status == 0))
        } else {
            None
        }
    }

    pub fn take_completion(&mut self) -> Completion {
        let status = self.status();
        let requested_len = self.urb().buffer_length as u32;
        let actual_len = self.urb().actual_length as usize;
        let len = match Direction::from_address(self.urb().endpoint) {
            Direction::Out => self.urb().buffer_length as u32,
            Direction::In => self.urb().actual_length as u32,
        };

        let mut empty = ManuallyDrop::new(Vec::new());
        let ptr = mem::replace(&mut self.urb_mut().buffer, empty.as_mut_ptr());
        let capacity = mem::replace(&mut self.capacity, 0);
        self.urb_mut().buffer_length = 0;
        self.urb_mut().actual_length = 0;
        let allocator = mem::replace(&mut self.allocator, Allocator::Default);

        Completion {
            status,
            actual_len,
            buffer: Buffer {
                ptr,
                len,
                requested_len,
                capacity,
                allocator,
            },
        }
    }

    #[inline]
    pub(super) fn urb(&self) -> &Urb {
        unsafe { &*self.urb }
    }

    #[inline]
    pub(super) fn urb_mut(&mut self) -> &mut Urb {
        unsafe { &mut *self.urb }
    }

    #[inline]
    pub(super) fn urb_ptr(&self) -> *mut Urb {
        self.urb
    }

    #[inline]
    pub fn status(&self) -> Result<(), TransferError> {
        if self.urb().status == 0 {
            return Ok(());
        }

        // It's sometimes positive, sometimes negative, but rustix panics if negative.
        Err(errno_to_transfer_error(Errno::from_raw_os_error(
            self.urb().status.abs(),
        )))
    }

    #[inline]
    pub fn control_in_data(&self) -> &[u8] {
        debug_assert!(self.urb().endpoint == 0x80);
        let urb = self.urb();
        unsafe {
            slice::from_raw_parts(
                urb.buffer.add(SETUP_PACKET_SIZE),
                urb.actual_length as usize,
            )
        }
    }
}

impl Pending<TransferData> {
    pub fn urb_ptr(&self) -> *mut Urb {
        // Get urb pointer without dereferencing as `TransferData`, because
        // it may be mutably aliased.
        unsafe { *addr_of_mut!((*self.as_ptr()).urb) }
    }
}

impl Drop for TransferData {
    fn drop(&mut self) {
        unsafe {
            drop(self.take_completion());
            drop(Box::from_raw(self.urb));
        }
    }
}
