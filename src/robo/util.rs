use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Error, ErrorKind, Read, Write};

use crate::RobotPose;
use crate::bresenham::{self, Bresenham, CircularBresenham};
use crate::controller::{CHUNK_L, NMap};
use crate::reeds_shepp::{Gear, RSPath, Steer};

pub fn save_mapgrid(path: &str, height: u32, width: u32, dx: f32, data: &[bool]) -> io::Result<()> {
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
    writer.flush()?;
    Ok(())
}

pub fn load_mapgrid(path: &str) -> io::Result<(u32, u32, f32, Vec<bool>)> {
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

pub fn save_nmap(path: &str, map: &NMap, dx: f32, l_chunk: usize) -> io::Result<()> {
    let n_chunks = map.len();
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(&(n_chunks as u64).to_le_bytes())?;
    writer.write_all(&dx.to_le_bytes())?;
    writer.write_all(&(l_chunk as u64).to_le_bytes())?;

    for (key, slice) in map.iter() {
        writer.write_all(&key.0.to_le_bytes())?;
        writer.write_all(&key.1.to_le_bytes())?;
        for &val in slice.iter() {
            writer.write_all(&val.to_le_bytes())?;
        }
    }
    writer.flush()?;
    Ok(())
}

pub fn load_nmap(path: &str) -> io::Result<(usize, usize, f32, NMap)> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut buf_4 = [0u8; 4];
    let mut buf_8 = [0u8; 8];

    reader.read_exact(&mut buf_8)?;
    let n_chunks = u64::from_le_bytes(buf_8) as usize;

    reader.read_exact(&mut buf_4)?;
    let dx = f32::from_le_bytes(buf_4);

    // Read and check chunmksize
    reader.read_exact(&mut buf_8)?;
    let l_chunk = u64::from_le_bytes(buf_8) as usize;
    if l_chunk != crate::controller::CHUNK_L {
        return Err(Error::new(
            ErrorKind::InvalidData,
            format!(
                "Chunk size mismatch: expected {}, got {}",
                crate::controller::CHUNK_L,
                l_chunk
            ),
        ));
    }

    let mut map = NMap::default();
    for _ in 0..n_chunks {
        let mut key = (0, 0);
        reader.read_exact(&mut buf_4)?;
        key.0 = i32::from_le_bytes(buf_4);
        reader.read_exact(&mut buf_4)?;
        key.1 = i32::from_le_bytes(buf_4);
        let mut chunk = [0.0f32; CHUNK_L * CHUNK_L];
        for val in &mut chunk {
            reader.read_exact(&mut buf_4)?;
            *val = f32::from_le_bytes(buf_4);
        }
        map.insert(key, chunk);
    }

    Ok((n_chunks, l_chunk, dx, map))
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

fn to_px(x: f32, y: f32, dx: f32) -> (i32, i32) {
    ((x / dx).floor() as i32, (y / dx).floor() as i32)
}

fn draw_px(buf: &mut [u32], w: usize, h: usize, x: i32, y: i32, color: u32) {
    if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
        buf[x as usize + y as usize * w] = color;
    }
}
pub fn draw_path(
    path: &RSPath,
    buf: &mut [u32],
    h: usize,
    w: usize,
    dx: f32,
    start_pose: RobotPose,
    turning_rad: f32,
    fwd_color: u32,
    reverse_color: u32,
) {
    let mut pose = start_pose;

    for seg in path {
        if seg.param <= 1e-6 {
            continue;
        }

        let s = match seg.gear {
            Gear::Forward => seg.param,
            Gear::Reverse => -seg.param,
        };

        let color = match seg.gear {
            Gear::Forward => fwd_color,
            Gear::Reverse => reverse_color,
        };

        let k = match seg.steer {
            Steer::Left => 1.0 / turning_rad,
            Steer::Right => -1.0 / turning_rad,
            Steer::Straight => 0.0,
        };

        let steps = ((seg.param / (0.5 * dx)).ceil() as usize).max(1);
        let mut prev_px = to_px(pose.x, pose.y, dx);

        for i in 1..=steps {
            let d = s * (i as f32 / steps as f32);
            let theta = pose.theta + k * d;

            let next = if k == 0.0 {
                RobotPose {
                    x: pose.x + d * pose.theta.cos(),
                    y: pose.y + d * pose.theta.sin(),
                    theta: pose.theta,
                }
            } else {
                RobotPose {
                    x: pose.x + (theta.sin() - pose.theta.sin()) / k,
                    y: pose.y + (pose.theta.cos() - theta.cos()) / k,
                    theta,
                }
            };

            let next_px = to_px(next.x, next.y, dx);

            for (x, y) in Bresenham::new(prev_px, next_px) {
                draw_px(buf, w, h, x, y, color);
            }
            draw_px(buf, w, h, next_px.0, next_px.1, color);

            prev_px = next_px;
        }

        let theta = pose.theta + k * s;
        pose = if k == 0.0 {
            RobotPose {
                x: pose.x + s * pose.theta.cos(),
                y: pose.y + s * pose.theta.sin(),
                theta: pose.theta,
            }
        } else {
            RobotPose {
                x: pose.x + (theta.sin() - pose.theta.sin()) / k,
                y: pose.y + (pose.theta.cos() - theta.cos()) / k,
                theta,
            }
        };
    }
}
pub fn draw_circle_segment(
    buf: &mut Vec<u32>,
    nw: usize,
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

    let is_valid_ang = |p: &(i32, i32)| -> bool {
        let cross_a = ax * p.1 - ay * p.0;
        let cross_b = bx * p.1 - by * p.0;
        if is_reflex {
            cross_a >= 0 || cross_b <= 0
        } else {
            cross_a >= 0 && cross_b <= 0
        }
    };

    for pts in bresenham {
        pts.iter().filter(|p| is_valid_ang(p)).for_each(|p| {
            buf[p.0 as usize + nw * (p.1 as usize)] = color;
        });
    }
}
pub fn draw_line(buf: &mut Vec<u32>, nw: usize, start: (i32, i32), end: (i32, i32), color: u32) {
    let bresenham = Bresenham::new(start, end);
    for p in bresenham {
        buf[p.0 as usize + nw * (p.1 as usize)] = color;
    }
}
