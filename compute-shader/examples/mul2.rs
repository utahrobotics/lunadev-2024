fn main() {
    env_logger::init();

    create_compute(include_wgsl!("mul2.wgsl"))
}