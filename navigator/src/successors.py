import math


bounds = 5
seen = set()

print("[")
for x in range(-bounds, bounds + 1):
    for y in range(-bounds, bounds + 1):
        if x == y == 0:
            continue
        # gcd = math.gcd(x, y)
        if (x ** 2 + y ** 2) ** 0.5 > bounds:
            continue
        # nx = int(x / gcd)
        # ny = int(y / gcd)
        # vec = (nx, ny)
        # if vec in seen:
        #     continue
        # seen.add(vec)
        print(f"\tVector2::new({x}, {y}),")
print("]")