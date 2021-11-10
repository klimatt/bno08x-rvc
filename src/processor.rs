use bbqueue::Producer;
use crate::BUFFER_SIZE;
use crate::Error;

pub struct Processor {
    producer: Producer<'static, BUFFER_SIZE>
}

impl Processor{
    pub fn new(producer: Producer<'static, BUFFER_SIZE>) -> Processor {
        Processor{
            producer
        }
    }

    pub fn process_slice(&mut self, slice: &[u8]) -> Result<(), Error>{
        match self.producer.grant_exact(slice.len()){
            Ok(mut wgr) => {
                wgr[0..slice.len()].copy_from_slice(slice);
                wgr.commit(slice.len());
                return Ok(());
            }
            Err(e) => {
                return Err(Error::BbqError(e))
            }
        }
    }
}