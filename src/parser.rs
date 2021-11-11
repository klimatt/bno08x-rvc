use bbqueue::{Consumer};
use crate::{BUFFER_SIZE, BNO08X_UART_RVC_HEADER, BNO08X_UART_RVC_FRAME_SIZE};
use crate::Error;
use core::borrow::{Borrow};
use serde::Deserialize;

#[derive(Deserialize, Debug, Copy, Clone)]
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
                            let data = &raw_bytes[idx..(idx + BNO08X_UART_RVC_FRAME_SIZE - 2)];
                            let csum = data[0..(data.len() - 1)].iter().map(|v| *v as u32).sum::<u32>() as u8;
                            let frame_unchecked: Bno08xRvcRawFrame = postcard::from_bytes(data).ok()?;
                            if csum == frame_unchecked.csum {
                                self.last_frame = Some( frame_unchecked );
                            }
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