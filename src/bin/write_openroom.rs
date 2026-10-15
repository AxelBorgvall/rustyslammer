use minifb::{Key, Window, WindowOptions};
use std::io::{self,  BufWriter,  Write};
use std::{cmp, fs::File};
#[allow(non_upper_case_globals)]

const dx: f32 = 0.05;
const L: usize = (20.0 / dx) as usize;


pub fn save_grid(
    path: &str,
    height: u32,
    width: u32,
    stepsize: f32,
    data: &[bool],
) -> io::Result<()> {
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

fn main() {
    let mut grid = vec![false; L * L];


    // Wall of edges
    grid.chunks_exact_mut(L).enumerate().for_each(|(y, row)| {
        if y == 0 || y == L - 1 {
            row.fill(true);
        } else {
            match row {
                [first, .., last] => {
                    *first = true;
                    *last = true;
                }
                [single] => {
                    *single = true;
                }
                [] => unreachable!(),
            }
        }
    });

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

	window.set_target_fps(10);

    while window.is_open() && !window.is_key_down(Key::Escape) {
        window.update_with_buffer(&buffer, L, L).unwrap();
    }

    save_grid("data/openroom.dat", L as u32, L as u32, dx, &grid).expect("GRAAAAHHHHH");
}
