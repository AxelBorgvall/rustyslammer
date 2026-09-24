#[derive(Clone)]
pub struct Bresenham {
    x: i32, y: i32,
    end_x: i32, end_y: i32,
    dx: i32, dy: i32,
    sx: i32, sy: i32,
    err: i32,
}

impl Bresenham {
    pub fn new(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Self {
        Self {
            x: start_x, y: start_y,
            end_x, end_y,
            dx: (start_x - end_x).abs(),
            dy: -(start_y - end_y).abs(),
            sx: if start_x < end_x { 1 } else { -1 },
            sy: if start_y < end_y { 1 } else { -1 },
            err: (start_x - end_x).abs() - (start_y - end_y).abs(),
        }
    }
}

impl Iterator for Bresenham {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<Self::Item> {
        let current = (self.x, self.y);
        
        if self.x == self.end_x && self.y == self.end_y {
            return None;
        }

        let e2 = 2 * self.err;
        if e2 >= self.dy { self.err += self.dy; self.x += self.sx; }
        if e2 <= self.dx { self.err += self.dx; self.y += self.sy; }

        Some(current)
    }
}