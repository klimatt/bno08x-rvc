use bbqueue::{Consumer, GrantR, BBBuffer};
use crate::{BUFFER_SIZE, MAX_AMOUNT_OF_FRAMES, BNO08X_UART_RVC_HEADER, BNO08X_UART_RVC_FRAME_SIZE};
use crate::Error;
use core::borrow::BorrowMut;

#[derive(Copy, Clone)]
pub struct Bno08xRvcRawFrame {
    index: u8,
    yaw: u16,
    pitch: u16,
    x_acc: u16,
    y_acc: u16,
    z_acc: u16,
    motion_intent: u8,
    motion_request: u8,
    rsvd: u8,
    csum: u8
}

enum State{
    LookingForFirstHeaderByte,
    LookingForSecondHeaderByte,
    GetFrameData
}

pub struct Parser{
    consumer: Consumer<'static, BUFFER_SIZE>,
    last_frame: Bno08xRvcRawFrame,
    state: State
}

impl Parser{
    pub fn new(consumer: Consumer<'static, BUFFER_SIZE>) -> Parser {
        Parser{
            consumer,
            last_frame: Bno08xRvcRawFrame {
                index: 0,
                yaw: 0,
                pitch: 0,
                x_acc: 0,
                y_acc: 0,
                z_acc: 0,
                motion_intent: 0,
                motion_request: 0,
                rsvd: 0,
                csum: 0
            },
            state: State::LookingForFirstHeaderByte
        }
    }

    pub fn worker<F: FnMut(&[Bno08xRvcRawFrame])>(&mut self, mut f: F) -> Result<(), Error> {
        let rgr = match self.consumer.read(){
            Ok(rgr) => { rgr }
            Err(e) => { return Err(Error::BbqError(e)) }
        };

        self.parse(rgr.as_ref()).unwrap();
        Ok(())
    }

    fn parse(&mut self, raw_bytes: &[u8]) -> Option<(Option<Bno08xRvcRawFrame>, usize)> { // release size
        if raw_bytes.len() >= BNO08X_UART_RVC_FRAME_SIZE {
            let mut rem_len = raw_bytes.len();
            for (idx, iter) in raw_bytes.iter().enumerate() {
                 match self.state {
                    State::LookingForFirstHeaderByte => {
                        if *iter == (BNO08X_UART_RVC_HEADER >> 8) as u8 {
                            self.state = State::LookingForSecondHeaderByte;
                        } else {
                            self.state = State::LookingForFirstHeaderByte;
                        }
                        idx += 1;
                    }
                    State::LookingForSecondHeaderByte => {
                        if *iter == BNO08X_UART_RVC_HEADER as u8 {
                            self.state = State::GetFrameData;
                        } else {
                            self.state = State::LookingForSecondHeaderByte;
                        }
                        idx += 1;
                    }
                    State::GetFrameData => {
                        if rem_len >= (BNO08X_UART_RVC_FRAME_SIZE - 2){
                            self.last_frame.index = *iter;
                            self.last_frame.yaw

                            idx += BNO08X_UART_RVC_FRAME_SIZE - 2;
                        }
                        else { idx += rem_len; }

                        self.state = State::LookingForFirstHeaderByte;
                        break;
                    }
                }
                rem_len -= 1;
            }
            if rem_len == 0{
                Some((None, idx))
            }
            else {
                Some((Some(frame), idx))
            }

        }
        else {
            None
        }
    }
}