use minifb::{Key, Window, WindowOptions};
use rand::random_range;
use std::f32::consts::PI;
use std::io::{self, BufWriter, Write};
use std::{cmp, fs::File};
#[allow(non_upper_case_globals)]
macro_rules! const_tan {
    ($angle:expr) => {{
        let x: f32 = $angle;
        let x2 = x * x;
        let x3 = x2 * x;
        let x5 = x3 * x2;
        let x7 = x5 * x2;
        let x9 = x7 * x2;

        x + (x3 / 3.0) + (2.0 * x5 / 15.0) + (17.0 * x7 / 315.0) + (62.0 * x9 / 2835.0)
    }};
}

const DX: f32 = 0.05;
const ROBO_W: f32 = 0.3;
const ROBO_L: f32 = 0.5;
// const WH_W: f32 = 0.15;
const WH_L: f32 = 0.35;
const MAX_WHANG: f32 = 40.0 * (PI / 180.0);
const TAN_WHANG: f32 = const_tan!(MAX_WHANG);
const TURN_RAD: f32 = WH_L / TAN_WHANG;
const CHUNK_L: f32 = (TURN_RAD + ROBO_W * 0.5 + ROBO_L * 0.5) * 1.2;

const CHUNK_NL: usize = (CHUNK_L / DX).ceil() as usize;
const N_CHUNKS: usize = 20;
const L: usize = N_CHUNKS * CHUNK_NL;

#[allow(non_camel_case_types)]
struct uPoint {
    x: usize,
    y: usize,
}

impl uPoint {
    fn from_tup(p: (usize, usize)) -> Self {
        Self { x: p.0, y: p.1 }
    }
    fn add(a: &uPoint, b: &uPoint) -> uPoint {
        uPoint {
            x: a.x + b.x,
            y: a.y + b.y,
        }
    }
}

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

fn del_rect(grid: &mut Vec<bool>, x0: uPoint, r: uPoint) {
    let x1 = uPoint::add(&x0, &r);

    for i in cmp::max(x0.x, 0)..cmp::min(x1.x, L) {
        for j in cmp::max(x0.y, 0)..cmp::min(x1.y, L) {
            grid[(j as usize) * L + i as usize] = false;
        }
    }
}
fn prim_pick(list: &Vec<(usize, usize)>) -> usize {
    random_range(0..list.len())
}
fn grow_tree_coo(pick: fn(&Vec<(usize, usize)>) -> usize) -> Vec<(usize, usize)> {
    let mut cells: Vec<(usize, usize)> = vec![];
    cells.push((random_range(0..N_CHUNKS), random_range(0..N_CHUNKS)));
    let mut visited: Vec<(usize, usize)> = cells.clone();
    let mut coo: Vec<(usize, usize)> = vec![];

    while cells.len() != 0 {
        let idx = pick(&cells);
        let cell = cells[idx];
        // Get neighbors
        let mut neighbors: Vec<(usize, usize)> = vec![];
        if cell.0 > 0 {
            let neighbor = (cell.0 - 1, cell.1);
            if !visited.contains(&neighbor) {
                neighbors.push(neighbor)
            }
        }
        if cell.1 > 0 {
            let neighbor = (cell.0, cell.1 - 1);
            if !visited.contains(&neighbor) {
                neighbors.push(neighbor)
            }
        }
        if cell.0 < N_CHUNKS - 1 {
            let neighbor = (cell.0 + 1, cell.1);
            if !visited.contains(&neighbor) {
                neighbors.push(neighbor)
            }
        }
        if cell.1 < N_CHUNKS - 1 {
            let neighbor = (cell.0, cell.1 + 1);
            if !visited.contains(&neighbor) {
                neighbors.push(neighbor)
            }
        }
        if neighbors.is_empty() {
            cells.remove(idx);
        } else {
            let neighbor = neighbors[random_range(0..neighbors.len())];
            cells.push(neighbor);
            visited.push(neighbor);
            coo.push((
                neighbor.0 + neighbor.1 * N_CHUNKS,
                cell.0 + cell.1 * N_CHUNKS,
            ));
        }
    }
    coo
}

fn main() {
    // Make grid of cells
    let mut grid = vec![false; L * L];
    for y in 0..L {
        for x in 0..L {
            if x % CHUNK_NL == 0
                || y % CHUNK_NL == 0
                || x % CHUNK_NL == CHUNK_NL - 1
                || y % CHUNK_NL == CHUNK_NL - 1
            {
                grid[y * L + x] = true;
            }
        }
    }

    let coo = grow_tree_coo(prim_pick);
    for (c1, c2) in coo {
        let x1 = c1 % N_CHUNKS * CHUNK_NL;
        let y1 = c1 / N_CHUNKS * CHUNK_NL;
        let x2 = c2 % N_CHUNKS * CHUNK_NL;
        let y2 = c2 / N_CHUNKS * CHUNK_NL;

        let corner = (x1.min(x2) + 2, y1.min(y2) + 2);

        let diff = {
            if x1 == x2 {
                (CHUNK_NL - 4, CHUNK_NL * 2 - 4)
            } else {
                (CHUNK_NL * 2 - 4, CHUNK_NL - 4)
            }
        };
        del_rect(&mut grid, uPoint::from_tup(corner), uPoint::from_tup(diff));
    }

    // outer wall
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

    save_grid("data/maze.dat", L as u32, L as u32, DX, &grid).expect("GRAAAAHHHHH");
}
