use crate::Error;
use crate::{BNO08X_UART_RVC_FRAME_SIZE, BNO08X_UART_RVC_HEADER, BUFFER_SIZE};
use bbqueue::Consumer;
use core::borrow::Borrow;
use serde::Deserialize;

#[derive(Deserialize, Debug, Copy, Clone, PartialEq)]
pub struct Bno08xRvcRawFrame {
    pub index: u8,
    pub yaw: i16,
    pub pitch: i16,
    pub roll: i16,
    pub x_acc: i16,
    pub y_acc: i16,
    pub z_acc: i16,
    pub motion_intent: u8,
    pub motion_request: u8,
    pub rsvd: u8,
    pub csum: u8,
}

const G_ACCELERATION: f32 = 9.80665;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Bno08xRvcPrettyFrame {
    pub index: u8,  // A monotonically increasing 8-bit count is provided (0-255) per report
    pub yaw: f32, // The yaw is a measure of the rotation around the Z-axis since reset. The yaw has a range of +/- 180̊  and is provided in 0.01̊  increments.
    pub pitch: f32, // The pitch is a measure of the rotation around the Y-axis. The pitch has a range of +/- 90̊  and is provided in 0.01̊  increments
    pub roll: f32, // The roll is a measure of the rotation around the X-axis. The roll has a range of +/- 180̊  and is provided in 0.01̊  increments
    pub x_acc: f32, // The acceleration along the X-axis, presented in m/s2
    pub y_acc: f32, // The acceleration along the Y-axis, presented in m/s2
    pub z_acc: f32, // The acceleration along the Z-axis, presented in m/s2
    pub motion_intent: u8, // BNO086 only. Otherwise, reserved
    pub motion_request: u8, // BNO086 only. Otherwise, reserved
    pub rsvd: u8, // The message is terminated with one (BNO086) or three (otherwise) reserved bytes, currently set to zero
}

impl Bno08xRvcRawFrame {
    fn convert(&self) -> Bno08xRvcPrettyFrame {
        Bno08xRvcPrettyFrame {
            index: self.index,
            yaw: (self.yaw as f32) / 100.0,
            pitch: (self.pitch as f32) / 100.0,
            roll: (self.roll as f32) / 100.0,
            x_acc: (self.x_acc as f32) * G_ACCELERATION / 1000.0,
            y_acc: (self.y_acc as f32) * G_ACCELERATION / 1000.0,
            z_acc: (self.z_acc as f32) * G_ACCELERATION / 1000.0,
            motion_intent: self.motion_intent,
            motion_request: self.motion_request,
            rsvd: self.rsvd,
        }
    }

    pub fn as_pretty_frame(&self) -> Bno08xRvcPrettyFrame {
        self.convert()
    }

    pub fn as_pretty_closure<F: FnMut(&Bno08xRvcPrettyFrame)>(&self, mut f: F) {
        f(&self.convert());
    }
}

#[derive(PartialEq)]
enum State {
    LookingForFirstHeaderByte,
    LookingForSecondHeaderByte,
    GetFrameData,
    GotFrame,
}

pub struct Parser {
    pub(crate) consumer: Consumer<'static, BUFFER_SIZE>,
    last_frame: Option<Bno08xRvcRawFrame>,
    state: State,
}

impl Parser {
    pub fn new(consumer: Consumer<'static, BUFFER_SIZE>) -> Parser {
        Parser {
            consumer,
            last_frame: None,
            state: State::LookingForFirstHeaderByte,
        }
    }

    pub fn get_last_raw_frame(&self) -> Option<Bno08xRvcRawFrame> {
        self.last_frame
    }

    pub fn worker<F: FnMut(&Bno08xRvcRawFrame)>(&mut self, mut f_opt: F) -> Result<(), Error> {
        return match self.consumer.split_read() {
            Err(e) => Err(Error::BbqError(e)),
            Ok(rgr) => {
                let (s1, s2) = rgr.bufs();
                let mut tmp = [0u8; BUFFER_SIZE];
                tmp[0..s1.len()].copy_from_slice(s1);
                tmp[s1.len()..s1.len() + s2.len()].copy_from_slice(s2);
                match self.parse(&tmp[0..(s1.len() + s2.len())]) {
                    None => Ok(()),
                    Some((frame_option, release_size)) => {
                        rgr.release(release_size);
                        match frame_option {
                            None => Ok(()),
                            Some(frame) => {
                                f_opt(frame.borrow());
                                Ok(())
                            }
                        }
                    }
                }
            }
        };
    }

    fn parse(&mut self, raw_bytes: &[u8]) -> Option<(Option<Bno08xRvcRawFrame>, usize)> {
        let mut release_size = 0;
        for (idx, iter) in raw_bytes.iter().enumerate() {
            match self.state {
                State::LookingForFirstHeaderByte => {
                    if *iter == (BNO08X_UART_RVC_HEADER >> 8) as u8 {
                        self.state = State::LookingForSecondHeaderByte;
                    } else {
                        self.state = State::LookingForFirstHeaderByte;
                        release_size = idx + 1;
                    }
                }
                State::LookingForSecondHeaderByte => {
                    if *iter == BNO08X_UART_RVC_HEADER as u8 {
                        self.state = State::GetFrameData;
                    } else {
                        self.state = State::LookingForFirstHeaderByte;
                        release_size = idx + 1;
                    }
                }
                State::GetFrameData => {
                    if raw_bytes.len() - idx >= (BNO08X_UART_RVC_FRAME_SIZE - 2) {
                        let data = &raw_bytes[idx..(idx + BNO08X_UART_RVC_FRAME_SIZE - 2)];
                        let csum = data[0..(data.len() - 1)]
                            .iter()
                            .map(|v| *v as u32)
                            .sum::<u32>() as u8;
                        let frame_unchecked: Bno08xRvcRawFrame = postcard::from_bytes(data).ok()?;
                        if csum == frame_unchecked.csum {
                            self.last_frame = Some(frame_unchecked);
                        }
                        release_size = idx + BNO08X_UART_RVC_FRAME_SIZE - 2;
                        self.state = State::GotFrame;
                    } else {
                        self.state = State::LookingForFirstHeaderByte;
                    }
                    break;
                }
                _ => {}
            }
        }
        if self.state == State::GotFrame {
            self.state = State::LookingForFirstHeaderByte;
            Some((self.last_frame, release_size))
        } else {
            self.state = State::LookingForFirstHeaderByte;
            Some((None, release_size))
        }
    }
}
