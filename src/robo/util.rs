use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};

use crate::bresenham::{self, Bresenham, CircularBresenham};

pub fn save_grid(path: &str, height: u32, width: u32, dx: f32, data: &[bool]) -> io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writer.write_all(&height.to_le_bytes())?;
    writer.write_all(&width.to_le_bytes())?;
    writer.write_all(&dx.to_le_bytes())?;
    let len = data.len() as u64;
    writer.write_all(&len.to_le_bytes())?;

    for &val in data {
        writer.write_all(&[val as u8])?;
    }
    Ok(())
}

pub fn load_data(path: &str) -> io::Result<(u32, u32, f32, Vec<bool>)> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut buf_4 = [0u8; 4];
    let mut buf_8 = [0u8; 8];

    reader.read_exact(&mut buf_4)?;
    let height = u32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_4)?;
    let width = u32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_4)?;
    let dx = f32::from_le_bytes(buf_4);

    reader.read_exact(&mut buf_8)?;
    let len = u64::from_le_bytes(buf_8) as usize;

    let mut data = Vec::with_capacity(len);
    let mut buf_1 = [0u8; 1]; // Buffer for reading 1 byte at a time

    for _ in 0..len {
        reader.read_exact(&mut buf_1)?;
        data.push(buf_1[0] != 0);
    }

    Ok((height, width, dx, data))
}

pub fn lerp_rgb(a: u32, b: u32, t: f32) -> u32 {
    let t = t.clamp(0.0, 1.0);

    let ar = ((a >> 16) & 0xFF) as f32;
    let ag = ((a >> 8) & 0xFF) as f32;
    let ab = (a & 0xFF) as f32;

    let br = ((b >> 16) & 0xFF) as f32;
    let bg = ((b >> 8) & 0xFF) as f32;
    let bb = (b & 0xFF) as f32;

    let r = (ar + (br - ar) * t) as u32;
    let g = (ag + (bg - ag) * t) as u32;
    let b = (ab + (bb - ab) * t) as u32;

    (r << 16) | (g << 8) | b
}

pub fn draw_circle_segment(
    buf: &mut Vec<u32>,
	nw:usize,
    center: (i32, i32),
    rad: i32,
    ang: (f32, f32),
    color: u32,
) {
    let mut bresenham = CircularBresenham::new(center, rad);

    let ax_f = ang.0.cos();
    let ay_f = ang.0.sin();
    let bx_f = ang.1.cos();
    let by_f = ang.1.sin();

    let is_reflex = (ax_f * by_f - ay_f * bx_f) < 0.0;

    // Evil, ugly ass fixed point math (completely unnecassary optimization)
    let ax = (ax_f * 1024.0) as i32;
    let ay = (ay_f * 1024.0) as i32;
    let bx = (bx_f * 1024.0) as i32;
    let by = (by_f * 1024.0) as i32;

    let is_valid_ang =|p: &(i32, i32)| -> bool {
        let cross_a = ax * p.1 - ay * p.0;
        let cross_b = bx * p.1 - by * p.0;
		if is_reflex{
			cross_a>=0||cross_b<=0
		} else {
			cross_a>=0&&cross_b<=0
		}
    };

    for pts in bresenham {
        pts.iter().filter(|p| is_valid_ang(p)).for_each(|p| {
			buf[p.0 as usize+nw*(p.1 as usize)]=color;
		});
    }
}
pub fn draw_line(
	buf: &mut Vec<u32>,
	nw:usize,
	start:(i32,i32),
	end:(i32,i32),
	color:u32,
){
	let bresenham=Bresenham::new(start,end);
	for p in bresenham{
		buf[p.0 as usize+nw*(p.1 as usize)]=color;
	}
}
