use bbqueue::{Consumer};
use crate::{BUFFER_SIZE, BNO08X_UART_RVC_HEADER, BNO08X_UART_RVC_FRAME_SIZE};
use crate::Error;
use core::borrow::{Borrow};

#[derive(Debug, Copy, Clone)]
pub struct Bno08xRvcRawFrame {
    index: u8,
    yaw: i16,
    pitch: i16,
    roll: i16,
    x_acc: i16,
    y_acc: i16,
    z_acc: i16,
    motion_intent: u8,
    motion_request: u8,
    rsvd: u8,
    csum: u8
}

enum State {
    LookingForFirstHeaderByte,
    LookingForSecondHeaderByte,
    GetFrameData
}

pub struct Parser{
    consumer: Consumer<'static, BUFFER_SIZE>,
    last_frame: Option<Bno08xRvcRawFrame>,
    state: State
}

impl Parser{
    pub fn new(consumer: Consumer<'static, BUFFER_SIZE>) -> Parser {
        Parser{
            consumer,
            last_frame: None,
            state: State::LookingForFirstHeaderByte
        }
    }

    pub fn get_last_raw_frame(&self) -> Option<Bno08xRvcRawFrame> {
        self.last_frame
    }

    pub fn worker<F: FnMut(&Bno08xRvcRawFrame)>(&mut self, mut f_opt: F) -> Result<(), Error> {
        return match self.consumer.read() {
         Ok(rgr) => {
             match self.parse(rgr.as_ref()) {
                 None => { Ok(()) }
                 Some((frame_option, release_size)) => {
                     rgr.release(release_size);
                     match frame_option {
                         None => { Ok(()) }
                         Some(frame) => {
                             f_opt(frame.borrow());
                             Ok(())
                         }
                     }
                 }
             }
         }
         Err(e) => { Err(Error::BbqError(e)) }
        };
    }

    fn parse(&mut self, raw_bytes: &[u8]) -> Option<(Option<Bno08xRvcRawFrame>, usize)> { // release size
        if raw_bytes.len() >= BNO08X_UART_RVC_FRAME_SIZE {
            let mut rem_len = raw_bytes.len();
            let mut release_size = raw_bytes.len();
            for (idx, iter) in raw_bytes.iter().enumerate() {
                 match self.state {
                    State::LookingForFirstHeaderByte => {
                        if *iter == (BNO08X_UART_RVC_HEADER >> 8) as u8 {
                            self.state = State::LookingForSecondHeaderByte;
                        } else {
                            self.state = State::LookingForFirstHeaderByte;
                        }
                    }
                    State::LookingForSecondHeaderByte => {
                        if *iter == BNO08X_UART_RVC_HEADER as u8 {
                            self.state = State::GetFrameData;
                        } else {
                            self.state = State::LookingForSecondHeaderByte;
                        }
                    }
                    State::GetFrameData => {
                        if rem_len >= (BNO08X_UART_RVC_FRAME_SIZE - 2){
                            self.last_frame = Some(
                                Bno08xRvcRawFrame{
                                    index: raw_bytes[idx],
                                    yaw: ((raw_bytes[idx + 2] as i16) << 8 ) | raw_bytes[idx + 1] as i16,
                                    pitch: ((raw_bytes[idx + 4] as i16) << 8 ) | raw_bytes[idx + 3] as i16,
                                    roll: ((raw_bytes[idx + 6] as i16) << 8 ) | raw_bytes[idx + 5] as i16,
                                    x_acc: ((raw_bytes[idx + 8] as i16) << 8 ) | raw_bytes[idx + 7] as i16,
                                    y_acc: ((raw_bytes[idx + 10] as i16) << 8 ) | raw_bytes[idx + 9] as i16,
                                    z_acc: ((raw_bytes[idx + 12] as i16) << 8 ) | raw_bytes[idx + 11] as i16,
                                    motion_intent: raw_bytes[idx + 13],
                                    motion_request: raw_bytes[idx + 14],
                                    rsvd: raw_bytes[idx + 15],
                                    csum: raw_bytes[idx + 16],
                                }
                            );
                            release_size = idx + BNO08X_UART_RVC_FRAME_SIZE - 2;
                        }
                        else { release_size = idx - 2; }
                        self.state = State::LookingForFirstHeaderByte;
                        break;
                    }
                }
                rem_len -= 1;
            }
            if rem_len == 0{
                self.state = State::LookingForFirstHeaderByte;
                Some((None, release_size))
            }
            else {
                Some((self.last_frame, release_size))
            }
        }
        else {
            None
        }
    }
}