use crate::robo::{ImuState, LidarScan, Map};
use std::{collections::HashMap, sync::{Arc, RwLock, atomic::AtomicBool}, thread};
use arc_swap::{ArcSwap};
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};

pub fn save_grid(path: &str, height: usize, width: usize, data: &[f32]) -> io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writer.write_all(&height.to_le_bytes())?;
    writer.write_all(&width.to_le_bytes())?;
    let len = data.len() as u64;
    writer.write_all(&len.to_le_bytes())?;

    for &val in data {
        writer.write_all(&val.to_le_bytes())?;
    }
    Ok(())
}

pub fn load_data(path: &str) -> io::Result<(u32, f32, Vec<f32>)> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut buf_4 = [0u8; 4];
    let mut buf_8 = [0u8; 8];
    reader.read_exact(&mut buf_4)?;
    let height = u32::from_le_bytes(buf_4);
    reader.read_exact(&mut buf_4)?;
    let width = f32::from_le_bytes(buf_4);
    reader.read_exact(&mut buf_8)?;
    let len = u64::from_le_bytes(buf_8) as usize;
    let mut data = Vec::with_capacity(len);
    for _ in 0..len {
        reader.read_exact(&mut buf_4)?;
        data.push(f32::from_le_bytes(buf_4));
    }

    Ok((height, width, data))
}

pub trait Environment: Send {
    fn spawn(&self, lidar_out:ArcSwap<LidarScan>);
}

pub struct SimEnv {
	// Robot
	pub n_rays:i32,
	pub spread:f32,
	pub speed:f32,
	pub angvel:f32,
	pub angles:Vec<f32>,

	// Map
	pub H:usize,
	pub W:usize,
	pub dx:f32,
	pub grid:Vec<bool>,
}

