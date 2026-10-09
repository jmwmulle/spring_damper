//! A single rough scalar-core timing, not a frame-rate claim.
#[allow(dead_code)]
#[path = "../src/convert.rs"]
mod convert;
#[allow(dead_code)]
#[path = "../src/spring.rs"]
mod spring;
fn main() {
    let mut states: [(f32, f32); 1000] =
        std::array::from_fn(|i| (i as f32 * 0.001, i as f32 * 0.0001));
    let start = std::time::Instant::now();
    for frame in 0..1000 {
        let goal = std::hint::black_box(if frame % 200 < 100 { 1.0 } else { -0.5 });
        for (x, v) in &mut states {
            spring::simple_spring_damper_exact(x, v, goal, std::hint::black_box(0.3), 1.0 / 60.0);
        }
        std::hint::black_box(&states);
    }
    println!(
        "1000 scalar springs × 1000 updates: {:.3} ms; {:.4} ms per batch of 1000",
        start.elapsed().as_secs_f64() * 1000.0,
        start.elapsed().as_secs_f64()
    );
}
