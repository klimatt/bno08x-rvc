#![no_std]

use bbqueue::{BBBuffer, Producer, Consumer};

pub mod processor;
pub mod parser;

const BNO08X_UART_RVC_FRAME_SIZE: usize = 19;
const BNO08X_UART_RVC_HEADER: u16 = 0xAAAA;
const MAX_AMOUNT_OF_FRAMES: usize = 3;

pub const BNO08X_UART_RVC_BAUD_RATE: u32 = 115_200;
pub const BUFFER_SIZE: usize = (BNO08X_UART_RVC_FRAME_SIZE * MAX_AMOUNT_OF_FRAMES ) + 1;


pub enum Error{
    BbqError(bbqueue::Error),
}

pub fn create(bbuffer: &'static BBBuffer<BUFFER_SIZE>) -> Result<(processor::Processor, parser::Parser), Error>{
        match bbuffer.try_split(){
            Ok((prod, cons)) => {
                Ok((
                       processor::Processor::new(prod),
                       parser::Parser::new(cons)
                ))
            }
            Err(e) => {
                Err(Error::BbqError(e))
            }
        }
}


