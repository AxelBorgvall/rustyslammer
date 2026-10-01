#[derive(Clone)]
pub struct Bresenham {
    x: i32, y: i32,
    end_x: i32, end_y: i32,
    dx: i32, dy: i32,
    sx: i32, sy: i32,
    err: i32,
}

impl Bresenham {
    pub fn new(start: (i32,i32), end: (i32, i32)) -> Self {
        Self {
            x: start.0, y: start.1,
            end_x:end.0, end_y:end.1,
            dx: (start.0 - end.0).abs(),
            dy: -(start.1 - end.1).abs(),
            sx: if start.0 < end.0 { 1 } else { -1 },
            sy: if start.1 < end.1 { 1 } else { -1 },
            err: (start.0 - end.0).abs() - (start.1 - end.1).abs(),
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