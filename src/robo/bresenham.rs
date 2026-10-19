#[derive(Clone, Copy)]
pub struct Bresenham {
    x: i32,
    y: i32,
    end_x: i32,
    end_y: i32,
    dx: i32,
    dy: i32,
    sx: i32,
    sy: i32,
    err: i32,
}

impl Bresenham {
    pub fn new(start: (i32, i32), end: (i32, i32)) -> Self {
        let dx = (start.0 - end.0).abs();
        let dy = -(start.1 - end.1).abs();
        Self {
            x: start.0,
            y: start.1,
            end_x: end.0,
            end_y: end.1,
            dx: (start.0 - end.0).abs(),
            dy: -(start.1 - end.1).abs(),
            sx: if start.0 < end.0 { 1 } else { -1 },
            sy: if start.1 < end.1 { 1 } else { -1 },
            err: dx + dy,
        }
    }
}

impl Iterator for Bresenham {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<Self::Item> {
		if self.sx==0 && self.sy==0{
			return None;
		}

        let current = (self.x, self.y);

        if self.x == self.end_x && self.y == self.end_y {
			self.sx=0;
			self.sy=0;
            return Some(current);
        }

        let e2 = 2 * self.err;
        if e2 >= self.dy {
            self.err += self.dy;
            self.x += self.sx;
        }
        if e2 <= self.dx {
            self.err += self.dx;
            self.y += self.sy;
        }

        Some(current)
    }
}

pub struct CircularBresenham {
    xc: i32,
    yc: i32,
    r: i32,
    x: i32,
    y: i32,
    d: i32,
}

impl CircularBresenham {
    pub fn new(center: (i32, i32), r: i32) -> Self {
        let (xc, yc) = center;
        Self {
            xc,
            yc,
            r,
            x: 0,
            y: r,
            d: 3 - 2 * r,
        }
    }
}

impl Iterator for CircularBresenham {
    type Item = [(i32, i32); 8];
    fn next(&mut self) -> Option<Self::Item> {
        if self.x <= self.y {
            let arr: [(i32, i32); 8] = [
                (self.xc + self.x, self.yc + self.y),
                (self.xc - self.x, self.yc + self.y),
                (self.xc + self.x, self.yc - self.y),
                (self.xc - self.x, self.yc - self.y),
                (self.xc + self.y, self.yc + self.x),
                (self.xc - self.y, self.yc + self.x),
                (self.xc + self.y, self.yc - self.x),
                (self.xc - self.y, self.yc - self.x),
            ];
            self.x += 1;
            if self.d < 0 {
                self.d = self.d + 4 * self.x + 6;
            } else {
                self.y -= 1;
                self.d = self.d + 4 * (self.x - self.y) + 10;
            }

            Some(arr)
        } else {
            None
        }
    }
}
