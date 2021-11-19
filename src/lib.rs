#![no_std]
#![deny(warnings)]

use bbqueue::BBBuffer;

pub mod parser;
pub mod processor;

const BNO08X_UART_RVC_FRAME_SIZE: usize = 19;
const BNO08X_UART_RVC_HEADER: u16 = 0xAAAA;
const MAX_AMOUNT_OF_FRAMES: usize = 3;

pub const BNO08X_UART_RVC_BAUD_RATE: u32 = 115_200;
pub const BUFFER_SIZE: usize = BNO08X_UART_RVC_FRAME_SIZE * MAX_AMOUNT_OF_FRAMES;

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
    use crate::parser::{Bno08xRvcPrettyFrame, Bno08xRvcRawFrame};
    use core::borrow::Borrow;

    const TEST_FRAME: Bno08xRvcRawFrame = Bno08xRvcRawFrame {
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
        csum: 0xE7,
    };

    #[test]
    fn parse_buffer_with_only_one_valid_frame() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|frame| {
            assert_eq!(*frame, TEST_FRAME);
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), Some(TEST_FRAME));
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            true
        );
    }

    #[test]
    fn parse_buffer_with_invalid_header() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [
            0xAA, 0xBB, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|_| {
            panic!();
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), None);
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            true
        );
    }

    #[test]
    fn parse_buffer_with_invalid_check_sum() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE] = [
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE9,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|_| {
            panic!();
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), None);
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            true
        );
    }

    #[test]
    fn parse_buffer_with_smaller_len() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE - 5] = [
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|_| {
            panic!();
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), None);
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            false
        );
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::GrantInProgress)),
            false
        );
        assert_eq!(parser_result.unwrap().len(), BNO08X_UART_RVC_FRAME_SIZE - 5);
    }

    #[test]
    fn parse_buffer_with_bigger_len() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE + 5] = [
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7, 0x01, 0x02, 0x03, 0x04, 0x05,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|frame| {
            assert_eq!(*frame, TEST_FRAME);
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), Some(TEST_FRAME));
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            false
        );
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::GrantInProgress)),
            false
        );
        assert_eq!(parser_result.unwrap().len(), 5);
    }

    #[test]
    fn parse_buffer_with_no_valid_frame() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE + 5] = [
            0xDD, 0xCA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7, 0x01, 0x02, 0x03, 0x04, 0x05,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|_| {
            panic!();
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), None);
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            true
        );
    }

    #[test]
    fn parse_buffer_with_valid_frame_inside_garbage() {
        let test_data: [u8; BNO08X_UART_RVC_FRAME_SIZE + 10] = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08,
            0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0x01, 0x02, 0x03, 0x04,
            0x05,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(matches!(processor_result, Ok(())), true);
        let worker_result = parser.worker(|frame| {
            assert_eq!(*frame, TEST_FRAME);
        });
        assert_eq!(matches!(worker_result, Ok(())), true);
        assert_eq!(parser.get_last_raw_frame(), Some(TEST_FRAME));
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            false
        );
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::GrantInProgress)),
            false
        );
        assert_eq!(parser_result.unwrap().len(), 5);
    }

    #[test]
    fn try_to_process_big_slice() {
        let test_data: [u8; BUFFER_SIZE + 10] = [0xFF; BUFFER_SIZE + 10];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let processor_result = processor.process_slice(&test_data);
        assert_eq!(
            matches!(
                processor_result,
                Err(Error::BbqError(bbqueue::Error::InsufficientSize))
            ),
            true
        );
        let worker_result = parser.worker(|_| {
            panic!();
        });
        assert_eq!(
            matches!(
                worker_result,
                Err(Error::BbqError(bbqueue::Error::InsufficientSize))
            ),
            true
        );
        assert_eq!(parser.get_last_raw_frame(), None);
        let parser_result = parser.consumer.read();
        assert_eq!(
            matches!(parser_result, Err(bbqueue::Error::InsufficientSize)),
            true
        );
    }

    #[test]
    fn try_to_process_invalid_frames_by_byte() {
        let test_data: [u8; BUFFER_SIZE] = [0xFF; BUFFER_SIZE];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        for (idx, iter) in test_data.iter().enumerate() {
            let byte = *iter;
            let processor_result = processor.process_slice(&[byte]);
            assert_eq!(
                matches!(
                    processor_result,
                    Err(Error::BbqError(bbqueue::Error::InsufficientSize))
                ),
                false
            );
            let worker_result = parser.worker(|_| {
                panic!();
            });
            assert_eq!(
                matches!(
                    worker_result,
                    Err(Error::BbqError(bbqueue::Error::InsufficientSize))
                ),
                false
            );
            assert_eq!(parser.get_last_raw_frame(), None);
            match parser.consumer.read() {
                Ok(_) => {
                    panic!("{:}", idx);
                }
                Err(e) => {
                    assert_eq!(matches!(e, bbqueue::Error::InsufficientSize), true);
                }
            }
        }
    }

    #[test]
    fn try_to_process_valid_frames_by_byte() {
        let test_data: [u8; BUFFER_SIZE] = [
            0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7, 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08,
            0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0xAA, 0xAA, 0xDE, 0x01,
            0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00,
            0xE7,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let mut flag_in_worker = 0;
        for iter in test_data.iter() {
            let byte = *iter;
            let processor_result = processor.process_slice(&[byte]);
            assert_eq!(
                matches!(
                    processor_result,
                    Err(Error::BbqError(bbqueue::Error::InsufficientSize))
                ),
                false
            );
            parser
                .worker(|frame| {
                    assert_eq!(*frame, TEST_FRAME);
                    flag_in_worker += 1;
                })
                .unwrap();
        }
        assert_eq!(flag_in_worker, 3);
        match parser.consumer.read() {
            Ok(_) => {
                panic!("Rgr is not empty!");
            }
            Err(e) => {
                assert_eq!(matches!(e, bbqueue::Error::InsufficientSize), true);
            }
        }
    }

    #[test]
    fn try_to_process_valid_and_invalid_frames_by_byte() {
        let test_data: [u8; BUFFER_SIZE] = [
            0xAA, 0xBB, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1,
            0x03, 0x00, 0x00, 0x00, 0xE7, 0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25, 0x08,
            0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00, 0xE7, 0xFF, 0xAA, 0xDE, 0x01,
            0x00, 0x92, 0xFF, 0x25, 0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00, 0x00, 0x00,
            0xE7,
        ];
        static BB: BBBuffer<{ BUFFER_SIZE }> = BBBuffer::new();
        let create_option = match create(BB.borrow()) {
            Ok((proc, pars)) => Some((proc, pars)),
            Err(_) => None,
        };
        assert_eq!(matches!(create_option, None), false);
        let (mut processor, mut parser) = create_option.unwrap();
        let mut flag_in_worker = 0;
        for iter in test_data.iter() {
            let byte = *iter;
            let processor_result = processor.process_slice(&[byte]);
            assert_eq!(
                matches!(
                    processor_result,
                    Err(Error::BbqError(bbqueue::Error::InsufficientSize))
                ),
                false
            );
            parser
                .worker(|frame| {
                    assert_eq!(*frame, TEST_FRAME);
                    flag_in_worker += 1;
                })
                .unwrap();
        }
        assert_eq!(flag_in_worker, 1);
        match parser.consumer.read() {
            Ok(_) => {
                panic!("Rgr is not empty!");
            }
            Err(e) => {
                assert_eq!(matches!(e, bbqueue::Error::InsufficientSize), true);
            }
        }
    }

    #[test]
    fn pretty_output_test() {
        const P_FRAME: Bno08xRvcPrettyFrame = Bno08xRvcPrettyFrame {
            index: 222,
            yaw: 0.01,
            pitch: -1.10,
            roll: 20.85,
            x_acc: -3.638,
            y_acc: -0.196,
            z_acc: 9.581,
            motion_intent: 0,
            motion_request: 0,
            rsvd: 0,
        };

        let frame = TEST_FRAME.as_pretty_frame();
        assert_eq!(frame.index, P_FRAME.index);
        assert!(
            (frame.yaw - P_FRAME.yaw).abs() < 0.001,
            "a = {}, b = {}",
            frame.yaw,
            P_FRAME.yaw
        );
        assert!(
            (frame.pitch - P_FRAME.pitch).abs() < 0.001,
            "a = {}, b = {}",
            frame.pitch,
            P_FRAME.pitch
        );
        assert!(
            (frame.roll - P_FRAME.roll).abs() < 0.001,
            "a = {}, b = {}",
            frame.roll,
            P_FRAME.roll
        );
        assert!(
            (frame.x_acc - P_FRAME.x_acc).abs() < 0.001,
            "a = {}, b = {}",
            frame.x_acc,
            P_FRAME.x_acc
        );
        assert!(
            (frame.y_acc - P_FRAME.y_acc).abs() < 0.001,
            "a = {}, b = {}",
            frame.y_acc,
            P_FRAME.y_acc
        );
        assert!(
            (frame.z_acc - P_FRAME.z_acc).abs() < 0.001,
            "a = {}, b = {}",
            frame.z_acc,
            P_FRAME.z_acc
        );
        assert_eq!(frame.motion_intent, P_FRAME.motion_intent);
        assert_eq!(frame.motion_request, P_FRAME.motion_request);
        assert_eq!(frame.rsvd, P_FRAME.rsvd);
    }
}
