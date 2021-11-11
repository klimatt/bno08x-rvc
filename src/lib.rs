#![no_std]
#![deny(warnings)]

use bbqueue::BBBuffer;

pub mod parser;
pub mod processor;

const BNO08X_UART_RVC_FRAME_SIZE: usize = 19;
const BNO08X_UART_RVC_HEADER: u16 = 0xAAAA;
const MAX_AMOUNT_OF_FRAMES: usize = 3;

pub const BNO08X_UART_RVC_BAUD_RATE: u32 = 115_200;
pub const BUFFER_SIZE: usize = (BNO08X_UART_RVC_FRAME_SIZE * MAX_AMOUNT_OF_FRAMES) + 1;

#[derive(Debug)]
pub enum Error {
    BbqError(bbqueue::Error),
}

pub fn create(
    bbuffer: &'static BBBuffer<BUFFER_SIZE>,
) -> Result<(processor::Processor, parser::Parser), Error> {
    match bbuffer.try_split() {
        Ok((prod, cons)) => Ok((processor::Processor::new(prod), parser::Parser::new(cons))),
        Err(e) => Err(Error::BbqError(e)),
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use core::borrow::Borrow;
    use crate::parser::Bno08xRvcRawFrame;

    const TEST_FRAME: Bno08xRvcRawFrame = Bno08xRvcRawFrame{
        index: 0xDE,
        yaw: 1i16,
        pitch: -110i16,
        roll: 2085i16,
        x_acc: -371i16,
        y_acc: -20i16,
        z_acc: 977i16,
        motion_intent: 0,
        motion_request: 0,
        rsvd: 0,
        csum: 0xE7
    };

    #[test]
    fn parse_buffer_with_only_one_valid_frame() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [ 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
            0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let result = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None
        };
        assert_eq!(matches!(result, None), false);
        let (mut processor, mut parser) = result.unwrap();
        processor.process_slice(&test_data).unwrap();
        parser.worker(|frame|{
            assert_eq!(*frame, TEST_FRAME);
        }).unwrap();
        assert_eq!(parser.get_last_raw_frame(), Some(TEST_FRAME));
    }

    #[test]
    fn parse_buffer_with_invalid_header() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [ 0xAA, 0xBB, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
            0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let result = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None
        };
        assert_eq!(matches!(result, None), false);
        let (mut processor, mut parser) = result.unwrap();
        processor.process_slice(&test_data).unwrap();
        parser.worker(|_|{
            panic!();
        }).unwrap();
        assert_eq!(parser.get_last_raw_frame(), None);
    }

    #[test]
    fn parse_buffer_with_invalid_check_sum() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [ 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
            0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE9];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let result = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None
        };
        assert_eq!(matches!(result, None), false);
        let (mut processor, mut parser) = result.unwrap();
        processor.process_slice(&test_data).unwrap();
        parser.worker(|_|{
            panic!();
        }).unwrap();
        assert_eq!(parser.get_last_raw_frame(), None);
    }

    #[test]
    fn parse_buffer_with_smaller_len() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE - 5] = [ 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
            0xFF, 0xD1];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let result = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None
        };
        assert_eq!(matches!(result, None), false);
        let (mut processor, mut parser) = result.unwrap();
        processor.process_slice(&test_data).unwrap();
        parser.worker(|_|{
            panic!();
        }).unwrap();
        assert_eq!(parser.get_last_raw_frame(), None);
    }

    #[test]
    fn parse_buffer_with_bigger_len() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE + 5] = [ 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC,
            0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0x01, 0x02, 0x03, 0x04, 0x05];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let result = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None
        };
        assert_eq!(matches!(result, None), false);
        let (mut processor, mut parser) = result.unwrap();
        processor.process_slice(&test_data).unwrap();
        parser.worker(|frame|{
            assert_eq!(*frame, TEST_FRAME);
        }).unwrap();
        assert_eq!(parser.get_last_raw_frame(), Some(TEST_FRAME));
    }
}
