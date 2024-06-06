use std::sync::{mpsc::Sender, Arc, Mutex, OnceLock};

use esp32_nimble::{uuid128, BLEAdvertisementData, BLEDevice, NimbleProperties};
use esp_idf_svc::{
    hal::{
        gpio::{self, PinDriver},
        peripherals::Peripherals,
        prelude::*,
        uart::{config::*, *},
    },
    io::Write,
};
use esp_idf_sys as _;

static WRITE_QUEUE: OnceLock<Sender<Vec<u8>>> = OnceLock::new();

fn main() -> Result<(), Box<dyn std::error::Error>> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take().unwrap();

    let tx = peripherals.pins.gpio2;
    let rx = peripherals.pins.gpio3;

    let mut re = PinDriver::output(peripherals.pins.gpio11).unwrap();

    re.set_low().unwrap();

    let device = BLEDevice::take();
    let ble_advertising = device.get_advertising();

    let config = esp_idf_svc::hal::uart::config::Config::new()
        .baudrate(Hertz(9600))
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::STOP1);

    let uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
    .unwrap();

    let uart = Arc::new(Mutex::new(uart));
    let uart_1 = uart.clone();

    let server = device.get_server();

    server.on_connect(|server, desc| {
        log::info!("Client connected: {:?}", desc);

        if server.connected_count() < (esp_idf_sys::CONFIG_BT_NIMBLE_MAX_CONNECTIONS as _) {
            log::info!("Multi-connect support: start advertising");
            ble_advertising.lock().start().unwrap();
        }
    });

    server.on_disconnect(|_desc, reason| {
        log::info!("Client disconnected ({:?})", reason);
    });

    let service = server.create_service(uuid128!("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"));

    let tx_characteristic = service.lock().create_characteristic(
        uuid128!("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
        NimbleProperties::READ,
    );

    let tx_characteristic_1 = tx_characteristic.clone();

    let rx_characteristic = service.lock().create_characteristic(
        uuid128!("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
        NimbleProperties::WRITE_NO_RSP,
    );

    let (tx, rx) = std::sync::mpsc::channel();

    WRITE_QUEUE.get_or_init(|| tx);

    rx_characteristic.lock().on_write(move |args| {
        if args.recv_data() == [b'\0'] {
            tx_characteristic_1.lock().set_value(&[]);
        } else {
            WRITE_QUEUE
                .get()
                .unwrap()
                .send(args.recv_data().to_vec())
                .unwrap();
        }
    });

    ble_advertising.lock().set_data(
        BLEAdvertisementData::new()
            .name("BLE Serial")
            .add_service_uuid(uuid128!("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")),
    )?;

    ble_advertising.lock().start()?;

    log::info!("bonded_addresses: {:?}", device.bonded_addresses());

    loop {
        let mut uart_1 = uart_1.lock().unwrap();
        let (mut uart_tx, uart_rx) = uart_1.split();

        if let Ok(data) = rx.try_recv() {
            if data.len() > 2 {
                re.set_high().unwrap();
                uart_tx.write_all(&data).unwrap();
                log::info!("write: {:02X?}", data);
                re.set_low().unwrap();
                let expected_head = data.split_at(2).0;
                let mut buf = [0; 255];
                let len = uart_rx.read(&mut buf[0..], 300).unwrap();

                if len != 0 {
                    let mut data = buf.split_at(len).0;
                    while !data.starts_with(expected_head) && data.len() > 0 {
                        data = &data[1..];
                    }
                    log::info!("read: {:02X?}, len: {len}", data);
                    tx_characteristic.lock().set_value(&data);
                }
            }
        }
    }
}
