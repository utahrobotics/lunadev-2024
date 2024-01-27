use nalgebra::Vector2;

pub(super) const SUCCESSORS: [(Vector2<isize>, f32); 8] = [
    (Vector2::new(-1, -1), 1.4142135623730951),
    (Vector2::new(-1, 0), 1.0),
    (Vector2::new(-1, 1), 1.4142135623730951),
    (Vector2::new(0, -1), 1.0),
    (Vector2::new(0, 1), 1.0),
    (Vector2::new(1, -1), 1.4142135623730951),
    (Vector2::new(1, 0), 1.0),
    (Vector2::new(1, 1), 1.4142135623730951),
];