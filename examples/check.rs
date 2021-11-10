use bno08x_rvc;
use bbqueue::BBBuffer;
use std::borrow::Borrow;
use bno08x_rvc::Error;
use bno08x_rvc::parser::Bno08xRvcRawFrame;


static BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();

fn main() -> std::io::Result<()> {

    let test_data: Vec<u8> = vec![0x34, 0x45, 0x32, 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0xFF, 0xBB, 0xAA];

    let (mut proc, mut parser) = match bno08x_rvc::create(BB.borrow()){
        Ok((proc, pars)) => {
            (proc, pars)
        }
        Err(e) => {
            println!("Can't create bno08x-rvc : {:?}", e);
            return Ok(());
        }
    };

    proc.process_slice(test_data.as_slice()).unwrap();

    println!("Get last raw frame: {:?}", parser.get_last_raw_frame());

    match parser.worker(|frame|{
        println!("Worker Frame: {:?}", frame);
    }) {
        Ok(_) => {}
        Err(e) => { println!("Worker error: {:?}", e)}
    }

    println!("Get last raw frame: {:?}", parser.get_last_raw_frame());
    Ok(())

}