use minifb::{Key, Window, WindowOptions};
use std::{cmp, fs::File, };
use std::io::{self, BufReader, BufWriter, Read, Write};
#[allow(non_upper_case_globals)]
const dx: f32 = 0.05;
const L: usize = (20.0 / dx) as usize;

#[allow(non_camel_case_types)]
struct iPoint {
    x: i32,
    y: i32,
}

impl iPoint {
    fn add(a: &iPoint, b: &iPoint) -> iPoint {
        iPoint {
            x: a.x + b.x,
            y: a.y + b.y,
        }
    }
}

pub fn save_grid(path: &str, height: u32, width: u32, stepsize: f32, data: &[bool]) -> io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writer.write_all(&height.to_le_bytes())?;
    writer.write_all(&width.to_le_bytes())?;
    writer.write_all(&stepsize.to_le_bytes())?;
    let len = data.len() as u64;
    writer.write_all(&len.to_le_bytes())?;

    for &val in data {
        writer.write_all(&[val as u8])?;
    }
    Ok(())
}
fn draw_rect(grid: &mut Vec<bool>, x0: iPoint, r: iPoint) {
    let x1 = iPoint::add(&x0, &r);

    for i in cmp::max(x0.x, 0)..cmp::min(x1.x, L as i32) {
        for j in cmp::max(x0.y, 0)..cmp::min(x1.y, L as i32) {
            grid[(j as usize) * L + i as usize] = true;
        }
    }
}

fn main() {
    let mut grid = vec![false; L * L];

    draw_rect(&mut grid, iPoint { x: 80, y: 80 }, iPoint { x: 200, y: 80 });
    draw_rect(&mut grid, iPoint { x: 300, y: 80 }, iPoint { x: 60, y: 80 });
    draw_rect(&mut grid, iPoint { x: 360, y: 0 }, iPoint { x: 40, y: 40 });
    draw_rect(&mut grid, iPoint { x: 80, y: 240 }, iPoint { x: 80, y: 80 });
    draw_rect(&mut grid, iPoint { x: 0, y: 280 }, iPoint { x: 120, y: 40 });
    draw_rect(
        &mut grid,
        iPoint { x: 220, y: 240 },
        iPoint { x: 120, y: 80 },
    );
    draw_rect(
        &mut grid,
        iPoint { x: 360, y: 360 },
        iPoint { x: 40, y: 40 },
    );
    draw_rect(&mut grid, iPoint { x: 0, y: 360 }, iPoint { x: 40, y: 40 });
    draw_rect(
        &mut grid,
        iPoint { x: 180, y: 368 },
        iPoint { x: 30, y: 32 },
    );

    let buffer: Vec<u32> = grid
        .iter()
        .map(|&is_filled| if is_filled { 0xFFFFFF } else { 0x000000 })
        .collect();

    let mut window = Window::new(
        "Grid Visualization - Press ESC to exit",
        L,
        L,
        WindowOptions::default(),
    )
    .unwrap_or_else(|e| {
        panic!("{}", e);
    });

    window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));

    while window.is_open() && !window.is_key_down(Key::Escape) {
        window.update_with_buffer(&buffer, L, L).unwrap();
    }
	
	save_grid("data/map1.dat", L as u32, L as u32, dx, &grid).expect("GRAAAAHHHHH");
	
}
