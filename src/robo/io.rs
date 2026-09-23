use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};

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