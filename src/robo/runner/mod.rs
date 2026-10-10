mod sim_runner;

pub use sim_runner::*;


/* ----------------------------- Rendering stuff ---------------------------- */
const PANEL_SIZE: usize = 900;
const WINDOW_WIDTH: usize = PANEL_SIZE * 2;
const WINDOW_HEIGHT: usize = PANEL_SIZE;
const BG_DARK: u32 = 0x323232;
const BG_LIGHT: u32 = 0x7F7F7F;

fn fit_to_canvas(
    src_data: &[u32],
    src_w: usize,
    src_h: usize,
    max_w: usize,
    max_h: usize,
    bg_color: u32,
) -> Vec<u32> {
    // Return and empty frame if the image is 0size
    if src_w == 0 || src_h == 0 {
        return vec![bg_color; max_w * max_h];
    }

    let scale = f64::min(max_w as f64 / src_w as f64, max_h as f64 / src_h as f64);
    let new_w = (src_w as f64 * scale) as usize;
    let new_h = (src_h as f64 * scale) as usize;

    let pad_left = (max_w - new_w) / 2;
    let pad_top = (max_h - new_h) / 2;

    let mut out = vec![bg_color; max_w * max_h];

    for dst_y in 0..new_h {
        for dst_x in 0..new_w {
            let src_x = (dst_x as f64 / scale) as usize;
            let src_y = (dst_y as f64 / scale) as usize;

            let src_x = src_x.min(src_w.saturating_sub(1));
            let src_y = src_y.min(src_h.saturating_sub(1));

            let out_idx = (dst_y + pad_top) * max_w + (dst_x + pad_left);
            let src_idx = src_y * src_w + src_x;

            out[out_idx] = src_data[src_idx];
        }
    }
    out
}

