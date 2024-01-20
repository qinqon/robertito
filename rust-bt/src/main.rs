use bluetooth_serial_port::{BtAddr, BtProtocol, BtSocket};
use gilrs::{ev::Axis, Event, EventType::AxisChanged, Gilrs};
use std::error::Error;
use std::io::Write;

fn main() {
    let mut gilrs = Gilrs::new().unwrap();
    // Iterate over all connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
        println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }
    let mut socket = connect().unwrap();
    let mut x: i16 = 0;
    let mut y: i16 = 0;
    loop {
        // Examine new events
        while let Some(Event {
            id: _,
            event,
            time: _,
        }) = gilrs.next_event()
        {
            match event {
                AxisChanged(Axis::LeftStickX, pos, _) => {
                    x = convert_event_pos(pos);
                    send_cmd(&mut socket, pos_cmd(x, y)).unwrap();
                }
                AxisChanged(Axis::LeftStickY, pos, _) => {
                    y = -convert_event_pos(pos);
                    send_cmd(&mut socket, pos_cmd(x, y)).unwrap();
                }
                _ => (),
            }
        }
    }
}

fn connect() -> Result<BtSocket, Box<dyn Error>> {
    // create and connect the RFCOMM socket
    let mut socket = BtSocket::new(BtProtocol::RFCOMM).unwrap();
    let address: BtAddr = "98:D3:31:F6:AF:06".parse().unwrap();
    socket.connect(address)?;
    Ok(socket)
}

fn convert_event_pos(pos: f32) -> i16 {
    let converted_pos = (255.0 * pos) as i16;
    if converted_pos.abs() < 25 {
        return 0;
    }
    converted_pos
}

fn pos_cmd(x: i16, y: i16) -> String {
    format!("<{x},{y}>")
}

fn send_cmd(socket: &mut BtSocket, cmd: String) -> Result<(), Box<dyn Error>> {
    println!("{cmd}");
    socket.write_all(cmd.as_bytes())?;
    Ok(())
}
