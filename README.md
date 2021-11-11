# `bno08x-rvc`
Small library to parse output from the BNO08x sensor (BNO080/85/86) in RVC mode.

Based on lockless queues: [bbqueue](https://crates.io/crates/bbqueue).

Datasheet on UART-RVC mode for BNO08x sensor: [Datasheet](https://github.com/klimatt/bno08x-rvc/blob/main/bno08x_rvc_datasheet.pdf)

## Example usage: 
```sh
use bbqueue::BBBuffer;
use bno08x_rvc;
use std::borrow::Borrow;

static BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();

fn main() -> std::io::Result<()> {
    let test_data: Vec<u8> = vec![
        0x34, 0x45, 0x32, 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
        0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0xFF, 0xBB, 0xAA,
    ];

    let (mut proc, mut parser) = match bno08x_rvc::create(BB.borrow()) {
        Ok((proc, pars)) => (proc, pars),
        Err(e) => {
            println!("Can't create bno08x-rvc : {:?}", e);
            return Ok(());
        }
    };

    proc.process_slice(test_data.as_slice()).unwrap();

    println!("Get last raw frame: {:?}", parser.get_last_raw_frame());

    match parser.worker(|frame| {
        println!("Worker Frame: {:?}", frame);
    }) {
        Ok(_) => {}
        Err(e) => {
            println!("Worker error: {:?}", e)
        }
    }

    println!("Get last raw frame: {:?}", parser.get_last_raw_frame());
    Ok(())
}
```
