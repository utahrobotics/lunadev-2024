import math


bounds = 1
seen = set()

print("[")
for x in range(-bounds, bounds + 1):
    for y in range(-bounds, bounds + 1):
        if x == y == 0:
            continue
        gcd = math.gcd(x, y)
        nx = int(x / gcd)
        ny = int(y / gcd)
        vec = (nx, ny)
        if vec in seen:
            continue
        seen.add(vec)
        print(f"\t(Vector2::new({nx}, {ny}), {(x ** 2 + y ** 2) ** 0.5}),")
print("]")