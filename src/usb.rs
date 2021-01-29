use rp2040_pac::{Interrupt, Peripherals};
use vcell::VolatileCell;

// USB DPRAM
const USB_DPRAM: *const () = 0x5010_0000 as _;
const USB_DPRAM_SIZE: usize = 4096;

struct EndpointBuffer {
    mem: &'static mut [VolatileCell<u8>],
}

const USB_NUM_ENDPOINTS: usize = 16;

// DPRAM content for a USB Device
#[repr(C)]
struct DpramContent {
    setup_packet: [u8; 4],
    ep_ctrl: [EpCtrl; USB_NUM_ENDPOINTS - 1],
    ep_buf_ctrl: [EpCtrl; USB_NUM_ENDPOINTS],

    ep0_buf_a: [u8; 0x40],
    ep0_buf_b: [u8; 0x40],

    exp_data: [u8; USB_DPRAM_SIZE - 0x180],
}

#[repr(C)]
struct EpCtrl {
    in_v: u32,
    out: u32,
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

    // TODO: Handler?
    handler: fn(*mut u8, u32),
    endpoint_control: Option<*const VolatileCell<u32>>,
    buffer_control: *const (),
    data_buffer: *const (),

    next_pid: u8,
}
struct UsbDeviceConfiguration {
    device_descriptor: &'static DeviceDescriptor,
    interface_descriptor: &'static InterfaceDescriptor,
    config_descriptor: &'static ConfigDescriptor,
    lang_descriptor: &'static [u8],
    descriptor_strings: &'static [&'static [u8]],
    //endpoints: &'static [EndpointConfig],
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

static USB_CONFIG: UsbDeviceConfiguration = unsafe {
    UsbDeviceConfiguration {
        device_descriptor: &USB_DEVICE_DESCRIPTOR,
        interface_descriptor: &USB_INTERFACE_DESCRIPTOR,
        config_descriptor: &USB_CONFIG_DESCRIPTOR,

        lang_descriptor: &LANG_DESCRIPTOR,
        descriptor_strings: &DESCRIPTOR_STRING,
    }
};

fn ep0_out_handler(buff: *mut u8, len: u32) {}
fn ep0_in_handler(buff: *mut u8, len: u32) {}
fn ep1_out_handler(buff: *mut u8, len: u32) {}
fn ep2_in_handler(buff: *mut u8, len: u32) {}

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

// See example in datasheet 4.1.3.2.1
unsafe fn usb_device_init(p: &mut Peripherals, cp: &mut rp2040_pac::CorePeripherals) {
    // Take USB controller out of reset
    p.RESETS.reset.modify(|_r, w| w.usbctrl().clear_bit());

    // Clear previous state in DPRAM
    // TODO!!!

    // Enable USB interrupt on this core
    cortex_m::peripheral::NVIC::unmask(Interrupt::USBCTRL_IRQ);

    // Mux the controller to the onboard usb phy
    p.USBCTRL_REGS
        .usb_muxing
        .modify(|_r, w| w.to_extphy().set_bit().softcon().set_bit());

    p.USBCTRL_REGS.usb_pwr.write(|w| {
        w.vbus_detect()
            .set_bit()
            .vbus_detect_override_en()
            .set_bit()
    });

    // Enable the USB controller in device mode
    p.USBCTRL_REGS
        .main_ctrl
        .write(|w| w.controller_en().set_bit());

    // Enable an interrupt per EP0 transaction
    p.USBCTRL_REGS
        .sie_ctrl
        .write(|w| w.ep0_int_1buf().set_bit());

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    p.USBCTRL_REGS.inte.write(|w| {
        w.buff_status()
            .set_bit()
            .bus_reset()
            .set_bit()
            .setup_req()
            .set_bit()
    });
    /*

    let USB_DPRAM_CONTENT: *const DpramContent = unsafe { USB_DPRAM as *const DpramContent };

    let endpoints = [
        EndpointConfig {
            descriptor: &EP0_OUT,
            handler: ep0_out_handler,
            endpoint_control: None,
            buffer_control: &(*USB_DPRAM_CONTENT).ep_buf_ctrl[0].out as *const u32 as _,
            data_buffer: &(*USB_DPRAM_CONTENT).ep0_buf_a[0] as *const u8 as _,
            next_pid: 0,
        },
        EndpointConfig {
            descriptor: &EP0_IN,
            handler: ep0_in_handler,
            endpoint_control: None,
            buffer_control: &(*USB_DPRAM_CONTENT).ep_buf_ctrl[0].in_v as *const u32 as _,
            data_buffer: &(*USB_DPRAM_CONTENT).ep0_buf_a[0] as *const u8 as _,
            next_pid: 0,
        },
        EndpointConfig {
            descriptor: &EP1_OUT,
            handler: ep1_out_handler,
            endpoint_control: Some(&(*USB_DPRAM_CONTENT).ep_ctrl[0].out as *const u32 as _),
            buffer_control: &(*USB_DPRAM_CONTENT).ep_buf_ctrl[1].out as *const u32 as _,
            data_buffer: &(*USB_DPRAM_CONTENT).exp_data[0 * 64] as *const u8 as _,
            next_pid: 0,
        },
        EndpointConfig {
            descriptor: &EP2_IN,
            handler: ep2_in_handler,
            endpoint_control: Some(&(*USB_DPRAM_CONTENT).ep_ctrl[1].out as *const u32 as _),
            buffer_control: &(*USB_DPRAM_CONTENT).ep_buf_ctrl[2].out as *const u32 as _,
            data_buffer: &(*USB_DPRAM_CONTENT).exp_data[1 * 64] as *const u8 as _,
            next_pid: 0,
        },
    ];

    */

    //usb_setup_endpoints(p, &endpoints);

    p.USBCTRL_REGS
        .sie_ctrl
        .modify(|_r, w| w.pullup_en().set_bit());
}

unsafe fn usb_setup_endpoints(p: &mut Peripherals, endpoints: &[EndpointConfig]) {
    for endpoint in endpoints {
        usb_setup_endpoint(endpoint);
    }
}

const EP_CTRL_ENABLE_BITS: u32 = 1 << 31;
const EP_CTRL_INTERRUPT_PER_BUFFER: u32 = 1 << 29;
const EP_CTRL_BUFFER_TYPE_LSB: u32 = 26;

unsafe fn usb_setup_endpoint(ep: &EndpointConfig) {
    // Not done for EP0
    if let Some(endpoint_control) = ep.endpoint_control {
        let dpram_offset = usb_buffer_offset(ep.data_buffer);
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

unsafe fn usb_handle_setup_packet() {}
