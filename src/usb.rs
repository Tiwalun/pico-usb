use core::{marker::PhantomData, ops::Deref, slice};

use vcell::VolatileCell;

// USB DPRAM
const USB_DPRAM: *const () = 0x5010_0000 as _;
const USB_DPRAM_SIZE: usize = 4096;

const USB_NUM_ENDPOINTS: usize = 16;

const USB_REQUEST_SET_ADDRESS: u8 = 0x05;
const USB_REQUEST_GET_DESCRIPTION: u8 = 0x06;
const USB_REQUEST_SET_CONFIGURATION: u8 = 0x09;

const USB_DT_DEVICE: u16 = 0x01;
const USB_DT_CONFIG: u16 = 0x02;
const USB_DT_STRING: u16 = 0x03;

const USB_BUF_CTRL_AVAIL: u32 = 0x400;
const USB_BUF_CTRL_FULL: u32 = 0x8000;
const USB_BUF_CTRL_DATA0_PID: u32 = 0x0000;
const USB_BUF_CTRL_DATA1_PID: u32 = 0x2000;

const USB_BUF_CTRL_LEN_MASK: u32 = 0x3ff;

// DPRAM content for a USB Device
pub struct DpramContent {
    setup_packet: [VolatileCell<u8>; 4],
    ep_ctrl: [EpCtrl; USB_NUM_ENDPOINTS - 1],
    ep_buf_ctrl: [EpCtrl; USB_NUM_ENDPOINTS],

    ep0_buf_a: [VolatileCell<u8>; 0x40],
    _ep0_buf_b: [VolatileCell<u8>; 0x40],

    exp_data: [VolatileCell<u8>; USB_DPRAM_SIZE - 0x180],
}

pub struct UsbDpram {
    _marker: PhantomData<*const ()>,
}
unsafe impl Send for UsbDpram {}
impl UsbDpram {
    #[inline(always)]
    pub const fn ptr() -> *const DpramContent {
        0x5010_0000 as *const _
    }
}
impl Deref for UsbDpram {
    type Target = DpramContent;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { &*UsbDpram::ptr() }
    }
}

struct EpCtrl {
    in_v: VolatileCell<u32>,
    out: VolatileCell<u32>,
}
#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
struct DeviceDescriptor {
    bLength: u8,
    bDescriptorType: u8,
    bcdUSB: u16,
    bDeviceClass: u8,
    bDeviceSubClass: u8,
    bDeviceProtocol: u8,
    bMaxPacketSize0: u8,
    idVendor: u16,
    idProduct: u16,
    bcdDevice: u16,
    iManufacturer: u8,
    iProduct: u8,
    iSerialNumber: u8,
    bNumConfigurations: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
struct ConfigDescriptor {
    bLength: u8,
    bDescriptorType: u8,
    wTotalLength: u16,
    bNumInterfaces: u8,
    bConfigurationValue: u8,
    iConfiguration: u8,
    bmAttributes: u8,
    bMaxPower: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
struct InterfaceDescriptor {
    bLength: u8,
    bDescriptorType: u8,
    bInterfaceNumber: u8,
    bAlternateSetting: u8,
    bNumEndpoints: u8,
    bInterfaceClass: u8,
    bInterfaceSubClass: u8,
    bInterfaceProtocol: u8,
    iInterface: u8,
}

#[repr(C)]
#[repr(packed)]
#[allow(non_snake_case)]
struct EndPointDescriptor {
    bLength: u8,
    bDescriptorType: u8,
    bEndpointAddress: u8,
    bmAttributes: u8,
    wMaxPacketSize: u16,
    bInterval: u8,
}

struct EndpointConfig {
    descriptor: &'static EndPointDescriptor,

    handler: fn(&mut UsbDevice, &[VolatileCell<u8>], u32),
    endpoint_control: Option<*const VolatileCell<u32>>,
    buffer_control: *const VolatileCell<u32>,
    data_buffer: &'static [VolatileCell<u8>],

    next_pid: u8,
}

impl EndpointConfig {
    fn is_tx(&self) -> bool {
        (self.descriptor.bEndpointAddress & USB_DIR_IN) == USB_DIR_IN
    }
}

struct UsbDeviceConfiguration {
    device_descriptor: &'static DeviceDescriptor,
    interface_descriptor: &'static InterfaceDescriptor,
    config_descriptor: &'static ConfigDescriptor,
    lang_descriptor: &'static [u8],
    descriptor_strings: &'static [&'static [u8]],

    endpoints: [Option<EndpointConfig>; USB_NUM_ENDPOINTS],
}

// Descriptors
static USB_DEVICE_DESCRIPTOR: DeviceDescriptor = DeviceDescriptor {
    bLength: core::mem::size_of::<DeviceDescriptor>() as u8,
    bDescriptorType: 1,    // USB_DT_DEVICE
    bcdUSB: 0x0110,        // USB 1.1 device
    bDeviceClass: 0,       // Specified in interface descriptor
    bDeviceSubClass: 0,    // No subclass
    bDeviceProtocol: 0,    // No protocol
    bMaxPacketSize0: 64,   // Max packet size for ep0
    idVendor: 0x0000,      // Your vendor id
    idProduct: 0x0001,     // Your product ID
    bcdDevice: 0,          // No device revision number
    iManufacturer: 1,      // Manufacturer string index
    iProduct: 2,           // Product string index
    iSerialNumber: 0,      // No serial number
    bNumConfigurations: 1, // One configuration
};

static USB_INTERFACE_DESCRIPTOR: InterfaceDescriptor = InterfaceDescriptor {
    bLength: core::mem::size_of::<InterfaceDescriptor>() as u8,
    bDescriptorType: 0x04, // USB_DT_INTERFACE
    bInterfaceNumber: 0,
    bAlternateSetting: 0,
    bNumEndpoints: 2,      // Interface has 2 endpoints
    bInterfaceClass: 0xff, // Vendor specific endpoint
    bInterfaceSubClass: 0,
    bInterfaceProtocol: 0,
    iInterface: 0,
};

static USB_CONFIG_DESCRIPTOR: ConfigDescriptor = ConfigDescriptor {
    bLength: core::mem::size_of::<ConfigDescriptor>() as u8,
    bDescriptorType: 2, // USB_DT_CONFIG,
    wTotalLength: (core::mem::size_of::<ConfigDescriptor>()
        + core::mem::size_of::<InterfaceDescriptor>()
        + core::mem::size_of::<EndPointDescriptor>()
        + core::mem::size_of::<EndPointDescriptor>()) as u16,
    bNumInterfaces: 1,
    bConfigurationValue: 1, // Configuration 1
    iConfiguration: 0,      // No string
    bmAttributes: 0xc0,     // attributes: self powered, no remote wakeup
    bMaxPower: 0x32,        // 100ma
};

// Intentionally empty
fn ep0_out_handler(_usb: &mut UsbDevice, _buff: &[VolatileCell<u8>], _len: u32) {}

fn ep0_in_handler(usb: &mut UsbDevice, _buff: &[VolatileCell<u8>], _len: u32) {
    if usb.should_set_address {
        usb.regs
            .addr_endp
            .write(|w| unsafe { w.address().bits(usb.dev_addr) });

        usb.should_set_address = false;
    } else {
        // Receive a zero length status protocol from the host
        usb.start_transfer(EP0_OUT_ADDR, 0, None);
    }
}

fn ep1_out_handler(usb: &mut UsbDevice, buff: &[VolatileCell<u8>], len: u32) {
    // Send data back to host

    // No packages larger than 64 bytes for now
    let mut local_buff = [0u8; 64];

    for i in 0..(len as usize) {
        local_buff[i] = buff[i].get();
    }

    usb.start_transfer(EP2_IN_ADDR, len, Some(&local_buff[..len as usize]))
}

// Get ready for 64 bytes from host
fn ep2_in_handler(usb: &mut UsbDevice, _buff: &[VolatileCell<u8>], _len: u32) {
    usb.start_transfer(EP1_OUT_ADDR, 64, None)
}

const LANG_DESCRIPTOR: [u8; 4] = [4, 0x03, 0x09, 0x04];

const DESCRIPTOR_STRING: &[&[u8]] = &["Raspberry Pi".as_bytes(), "Pico Test Device".as_bytes()];

const USB_DIR_OUT: u8 = 0;
const USB_DIR_IN: u8 = 0x80;

const USB_TRANSFER_TYPE_CONTROL: u8 = 2;
const USB_TRANSFER_TYPE_BULK: u8 = 2;

const USB_DT_ENDPOINT: u8 = 5;

const EP0_OUT_ADDR: u8 = USB_DIR_OUT | 0;
const EP0_IN_ADDR: u8 = USB_DIR_IN | 0;
const EP1_OUT_ADDR: u8 = USB_DIR_OUT | 1;
const EP2_IN_ADDR: u8 = USB_DIR_IN | 2;

static EP0_OUT: EndPointDescriptor = EndPointDescriptor {
    bLength: core::mem::size_of::<EndPointDescriptor>() as u8,
    bDescriptorType: USB_DT_ENDPOINT,
    bEndpointAddress: EP0_OUT_ADDR, // EP number 1, OUT from host (rx to device)
    bmAttributes: USB_TRANSFER_TYPE_CONTROL,
    wMaxPacketSize: 64,
    bInterval: 0,
};

static EP0_IN: EndPointDescriptor = EndPointDescriptor {
    bLength: core::mem::size_of::<EndPointDescriptor>() as u8,
    bDescriptorType: USB_DT_ENDPOINT,
    bEndpointAddress: EP0_IN_ADDR, // EP number 1, OUT from host (rx to device)
    bmAttributes: USB_TRANSFER_TYPE_CONTROL,
    wMaxPacketSize: 64,
    bInterval: 0,
};

static EP1_OUT: EndPointDescriptor = EndPointDescriptor {
    bLength: core::mem::size_of::<EndPointDescriptor>() as u8,
    bDescriptorType: USB_DT_ENDPOINT,
    bEndpointAddress: EP1_OUT_ADDR, // EP number 1, OUT from host (rx to device)
    bmAttributes: USB_TRANSFER_TYPE_BULK,
    wMaxPacketSize: 64,
    bInterval: 0,
};

static EP2_IN: EndPointDescriptor = EndPointDescriptor {
    bLength: core::mem::size_of::<EndPointDescriptor>() as u8,
    bDescriptorType: USB_DT_ENDPOINT,
    bEndpointAddress: EP2_IN_ADDR, // EP number 2, IN from host (tx from device)
    bmAttributes: USB_TRANSFER_TYPE_BULK,
    wMaxPacketSize: 64,
    bInterval: 0,
};

pub struct UsbDevice {
    config: UsbDeviceConfiguration,

    regs: rp2040_pac::USBCTRL_REGS,

    dpram: UsbDpram,

    configured: bool,

    dev_addr: u8,
    should_set_address: bool,
}

impl UsbDevice {
    fn new(config: UsbDeviceConfiguration, regs: rp2040_pac::USBCTRL_REGS) -> Self {
        let dpram = UsbDpram {
            _marker: PhantomData,
        };

        Self {
            config,
            configured: false,
            dev_addr: 0,
            should_set_address: false,
            dpram,
            regs,
        }
    }

    // Run USB
    pub fn poll(&mut self) {
        let raw_interrupts = self.regs.intr.read();

        if raw_interrupts.setup_req().bit_is_set() {
            self.regs
                .sie_status
                .modify(|_r, w| w.setup_rec().clear_bit());
            self.handle_setup_packet()
        }

        if raw_interrupts.buff_status().bit_is_set() {
            self.handle_buffer()
        }

        if raw_interrupts.bus_reset().bit_is_set() {
            self.regs
                .sie_status
                .modify(|_r, w| w.bus_reset().clear_bit());
            self.bus_reset();
        }
    }

    fn get_endpoint_configuration_mut(
        &mut self,
        endpoint_address: u8,
    ) -> Option<&mut EndpointConfig> {
        self.config
            .endpoints
            .iter_mut()
            .filter_map(|c| c.as_mut())
            .find(|cfg| cfg.descriptor.bEndpointAddress == endpoint_address)
    }

    pub fn configured(&self) -> bool {
        self.configured
    }

    fn handle_setup_packet(&mut self) {
        // read packet (first 8 bytes from DPRAM)
        let setup_packet = UsbSetupPacket::from_raw(&self.dpram.setup_packet);

        let mut ep0_in_config = self.get_endpoint_configuration_mut(EP0_IN_ADDR).unwrap();
        ep0_in_config.next_pid = 1;

        let req_direction = setup_packet.bmRequestType;
        let req = setup_packet.bRequest;

        match req_direction {
            USB_DIR_OUT => match req {
                USB_REQUEST_SET_ADDRESS => self.set_device_address(&setup_packet),
                USB_REQUEST_SET_CONFIGURATION => self.set_device_configuration(&setup_packet),
                _ => (), // Other request
            },
            USB_DIR_IN => {
                if req == USB_REQUEST_GET_DESCRIPTION {
                    let descriptor_type = setup_packet.wValue >> 8;

                    match descriptor_type {
                        USB_DT_DEVICE => self.handle_device_descriptor(&setup_packet),
                        USB_DT_CONFIG => self.handle_config_descriptor(&setup_packet),
                        USB_DT_STRING => self.handle_string_descriptor(&setup_packet),
                        _ => (), // Ignore other descriptor types
                    }
                }
            }
            _ => (), // Ignore other requests
        }
    }

    fn set_device_address(&mut self, packet: &UsbSetupPacket) {
        // We have to send a 0 length status packet first, with address 0
        self.dev_addr = (packet.wValue & 0xff) as u8;
        self.should_set_address = true;

        self.start_transfer(EP0_IN_ADDR, 0, None);
    }

    fn set_device_configuration(&mut self, _packet: &UsbSetupPacket) {
        // Just acknowledge
        self.start_transfer(EP0_IN_ADDR, 0, None);
        self.configured = true;
    }

    fn handle_device_descriptor(&mut self, _packet: &UsbSetupPacket) {
        let device_descriptor = self.config.device_descriptor;
        let mut ep = self.get_endpoint_configuration_mut(EP0_IN_ADDR).unwrap();

        ep.next_pid = 1;

        let data = unsafe {
            slice::from_raw_parts(
                (device_descriptor as *const DeviceDescriptor) as *const u8,
                core::mem::size_of::<DeviceDescriptor>(),
            )
        };

        self.start_transfer(EP0_IN_ADDR, data.len() as u32, Some(data))
    }

    fn handle_config_descriptor(&mut self, packet: &UsbSetupPacket) {
        let config_descriptor = self.config.config_descriptor;

        let mut buff = [0u8; 64];

        let mut buf_index = 0;

        let data = unsafe {
            slice::from_raw_parts(
                (config_descriptor as *const ConfigDescriptor) as *const u8,
                core::mem::size_of::<ConfigDescriptor>(),
            )
        };

        let data_len = data.len();

        buff[..data_len].copy_from_slice(data);
        buf_index += data_len;

        // If we more than just the config descriptor copy it all
        if packet.wLength >= config_descriptor.wTotalLength {
            // copy interface descriptor
            let interface_descriptor = self.config.interface_descriptor;

            let data = unsafe {
                slice::from_raw_parts(
                    (interface_descriptor as *const InterfaceDescriptor) as *const u8,
                    core::mem::size_of::<InterfaceDescriptor>(),
                )
            };
            let data_len = data.len();

            buff[buf_index..buf_index + data_len].copy_from_slice(data);
            buf_index += data_len;

            for i in 2..USB_NUM_ENDPOINTS {
                if let Some(endpoint) = &self.config.endpoints[i] {
                    // Copy endpoint descriptor
                    let endpoint_descriptor = endpoint.descriptor;

                    let data = unsafe {
                        slice::from_raw_parts(
                            (endpoint_descriptor as *const EndPointDescriptor) as *const u8,
                            core::mem::size_of::<EndPointDescriptor>(),
                        )
                    };
                    let data_len = data.len();

                    buff[buf_index..buf_index + data_len].copy_from_slice(data);
                    buf_index += data_len;
                }
            }
        }

        let data_len = buf_index;

        self.start_transfer(EP0_IN_ADDR, data_len as u32, Some(&buff[..data_len]))
    }

    fn handle_string_descriptor(&mut self, packet: &UsbSetupPacket) {
        let i = packet.wValue & 0xff;

        let mut ep_buffer = [0u8; 64];

        let len = if i == 0 {
            ep_buffer[..4].copy_from_slice(&self.config.lang_descriptor[..4]);
            4
        } else {
            usb_prepare_string_descriptor(
                &mut ep_buffer,
                self.config.descriptor_strings[(i - 1) as usize],
            )
        };

        self.start_transfer(EP0_IN_ADDR, len as u32, Some(&ep_buffer[..len]))
    }

    fn handle_buffer(&mut self) {
        let buffers = self.regs.buff_status.read().bits();
        let mut remaining_buffers = buffers;

        let mut bit = 1;

        for i in 0..(2 * USB_NUM_ENDPOINTS) {
            if remaining_buffers & bit == bit {
                unsafe {
                    self.regs.buff_status.modify(|r, w| {
                        let mut bits = r.bits();
                        bits &= !bit;
                        w.bits(bits)
                    })
                };

                self.handle_buff_done((i >> 1) as u8, i % 2 == 0);

                remaining_buffers &= !bit;

                if remaining_buffers == 0 {
                    break;
                }
            }

            bit <<= 1;
        }
    }

    fn handle_buff_done(&mut self, endpoint_index: u8, input: bool) {
        let ep_addr = endpoint_index | if input { USB_DIR_IN } else { 0 };

        if let Some(endpoint) = self
            .config
            .endpoints
            .iter()
            .filter_map(|ec| ec.as_ref())
            .find(|ep| ep.descriptor.bEndpointAddress == ep_addr)
        {
            let buffer_control = unsafe { (*endpoint.buffer_control).get() };

            let len = buffer_control & USB_BUF_CTRL_LEN_MASK;

            let buffer = endpoint.data_buffer;

            (endpoint.handler)(self, buffer, len);
            return;
        }
    }

    fn bus_reset(&mut self) {
        self.dev_addr = 0;
        self.should_set_address = false;

        unsafe { self.regs.addr_endp.write_with_zero(|w| w.bits(0)) }

        self.configured = false;
    }

    fn start_transfer(&mut self, endpoint_addr: u8, len: u32, data: Option<&[u8]>) {
        assert!(len <= 64);

        let mut val = len as u32 | USB_BUF_CTRL_AVAIL;

        let ep_config = self.get_endpoint_configuration_mut(endpoint_addr).unwrap();

        if let Some(data) = data {
            if ep_config.is_tx() {
                // Copy data into USB memory

                for (index, byte) in data.iter().enumerate() {
                    ep_config.data_buffer[index].set(*byte);
                }

                // Mark buffer as full
                val |= USB_BUF_CTRL_FULL;
            }
        }

        val |= if ep_config.next_pid == 1 {
            USB_BUF_CTRL_DATA1_PID
        } else {
            USB_BUF_CTRL_DATA0_PID
        };

        ep_config.next_pid ^= 1;

        unsafe { (*ep_config.buffer_control).set(val) }
    }
}

struct UsbSetupPacket {
    bmRequestType: u8,
    bRequest: u8,
    wValue: u16,
    wIndex: u16,
    wLength: u16,
}

impl UsbSetupPacket {
    fn from_raw(bytes: &[VolatileCell<u8>]) -> Self {
        Self {
            bmRequestType: bytes[0].get(),
            bRequest: bytes[1].get(),
            wValue: (bytes[2].get() as u16) | (bytes[3].get() as u16) << 8,
            wIndex: (bytes[4].get() as u16) | (bytes[5].get() as u16) << 8,
            wLength: (bytes[6].get() as u16) | (bytes[7].get() as u16) << 8,
        }
    }
}

// See example in datasheet 4.1.3.2.1
pub unsafe fn usb_device_init(
    reset_control: &rp2040_pac::RESETS,
    usb_registers: rp2040_pac::USBCTRL_REGS,
) -> UsbDevice {
    // Take USB controller out of reset
    reset_control.reset.modify(|_r, w| w.usbctrl().clear_bit());

    // Clear previous state in DPRAM
    let usb_dpram_content: *const DpramContent = USB_DPRAM as *const DpramContent;
    // TODO: actually clear DPRAM

    // Disabled, polling is used for now (Enable USB interrupt on this core)
    // cortex_m::peripheral::NVIC::unmask(Interrupt::USBCTRL_IRQ);

    // Mux the controller to the onboard usb phy
    usb_registers
        .usb_muxing
        .modify(|_r, w| w.to_phy().set_bit().softcon().set_bit());

    usb_registers.usb_pwr.write(|w| {
        w.vbus_detect()
            .set_bit()
            .vbus_detect_override_en()
            .set_bit()
    });

    // Enable the USB controller in device mode
    usb_registers
        .main_ctrl
        .write(|w| w.controller_en().set_bit().host_ndevice().clear_bit());

    // Enable an interrupt per EP0 transaction
    usb_registers.sie_ctrl.write(|w| w.ep0_int_1buf().set_bit());

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    // usb_registers.inte.write(|w| {
    //     w.buff_status()
    //         .set_bit()
    //         .bus_reset()
    //         .set_bit()
    //         .setup_req()
    //         .set_bit()
    // });

    let endpoints = [
        Some(EndpointConfig {
            descriptor: &EP0_OUT,
            handler: ep0_out_handler,
            endpoint_control: None,
            buffer_control: &(*usb_dpram_content).ep_buf_ctrl[0].out as *const _,
            data_buffer: slice::from_raw_parts((*usb_dpram_content).ep0_buf_a.as_ptr(), 0x40),
            next_pid: 0,
        }),
        Some(EndpointConfig {
            descriptor: &EP0_IN,
            handler: ep0_in_handler,
            endpoint_control: None,
            buffer_control: &(*usb_dpram_content).ep_buf_ctrl[0].in_v as *const _,
            data_buffer: slice::from_raw_parts((*usb_dpram_content).ep0_buf_a.as_ptr(), 0x40),

            next_pid: 0,
        }),
        Some(EndpointConfig {
            descriptor: &EP1_OUT,
            handler: ep1_out_handler,
            endpoint_control: Some(&(*usb_dpram_content).ep_ctrl[0].out as *const _),
            buffer_control: &(*usb_dpram_content).ep_buf_ctrl[1].out as *const _,
            data_buffer: slice::from_raw_parts((*usb_dpram_content).exp_data.as_ptr(), 0x40),

            next_pid: 0,
        }),
        Some(EndpointConfig {
            descriptor: &EP2_IN,
            handler: ep2_in_handler,
            endpoint_control: Some(&(*usb_dpram_content).ep_ctrl[1].out as *const _),
            buffer_control: &(*usb_dpram_content).ep_buf_ctrl[2].out as *const _,
            data_buffer: slice::from_raw_parts(
                (*usb_dpram_content).exp_data.as_ptr().add(0x40),
                0x40,
            ),

            next_pid: 0,
        }),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    ];

    usb_setup_endpoints(&endpoints);

    let usb_config: UsbDeviceConfiguration = UsbDeviceConfiguration {
        device_descriptor: &USB_DEVICE_DESCRIPTOR,
        interface_descriptor: &USB_INTERFACE_DESCRIPTOR,
        config_descriptor: &USB_CONFIG_DESCRIPTOR,

        lang_descriptor: &LANG_DESCRIPTOR,
        descriptor_strings: &DESCRIPTOR_STRING,

        endpoints,
    };

    usb_registers
        .sie_ctrl
        .modify(|_r, w| w.pullup_en().set_bit());

    UsbDevice::new(usb_config, usb_registers)
}

unsafe fn usb_setup_endpoints(endpoints: &[Option<EndpointConfig>]) {
    for endpoint in endpoints {
        if let Some(endpoint) = endpoint {
            usb_setup_endpoint(endpoint);
        }
    }
}

const EP_CTRL_ENABLE_BITS: u32 = 1 << 31;
const EP_CTRL_INTERRUPT_PER_BUFFER: u32 = 1 << 29;
const EP_CTRL_BUFFER_TYPE_LSB: u32 = 26;

unsafe fn usb_setup_endpoint(ep: &EndpointConfig) {
    // Not done for EP0
    if let Some(endpoint_control) = ep.endpoint_control {
        let dpram_offset = usb_buffer_offset(ep.data_buffer.as_ptr() as _);
        let reg = EP_CTRL_ENABLE_BITS
            | EP_CTRL_INTERRUPT_PER_BUFFER
            | ((ep.descriptor.bmAttributes as u32) << EP_CTRL_BUFFER_TYPE_LSB)
            | dpram_offset;

        (*endpoint_control).set(reg);
    }
}

fn usb_buffer_offset(dpram_pointer: *const ()) -> u32 {
    let pointer_value = dpram_pointer as u32;

    pointer_value - USB_DPRAM as u32
}

fn usb_prepare_string_descriptor(buffer: &mut [u8], descriptor: &[u8]) -> usize {
    let bLength = 2 + (descriptor.len() * 2);
    let bDescriptorType = 0x03;

    let mut buf_index = 0;

    buffer[buf_index] = bLength as u8;
    buf_index += 1;

    buffer[buf_index] = bDescriptorType;
    buf_index += 1;

    let mut descriptor_index = 0;

    while descriptor_index < descriptor.len() {
        let c = descriptor[descriptor_index];
        descriptor_index += 1;

        buffer[buf_index] = c;
        buf_index += 1;

        buffer[buf_index] = 0;
        buf_index += 1;
    }

    bLength
}
