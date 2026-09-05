"""Check the GLSL bitplane formula against independent Cartesian Morton masks."""

from itertools import product


UINT32_MAX = (1 << 32) - 1


def direction_reach_mask(octant, position):
    # Mirror directionReachMask's four lanes and uint32 wraparound.
    masks = []
    for axis, bit0, bit1 in (
        (0, 0xAAAAAAAA, 0xFF00FF00),
        (1, 0xCCCCCCCC, 0xFFFF0000),
        (2, 0xF0F0F0F0, 0x00000000),
        (2, 0xF0F0F0F0, 0xFFFFFFFF),
    ):
        flip = (-((octant >> axis) & 1)) & UINT32_MAX
        q = ((position[axis] & UINT32_MAX) ^ flip) & 3
        low = (bit0 ^ flip) | (((q & 1) - 1) & UINT32_MAX)
        high = bit1 ^ flip
        select_or = ((q >> 1) - 1) & UINT32_MAX
        masks.append((low & high) | ((low | high) & select_or))
    xy = masks[0] & masks[1]
    return xy & masks[2], xy & masks[3]


def cartesian_mask(lo, hi):
    mask = 0
    for cell in product(*(range(lo[a], hi[a] + 1) for a in range(3))):
        morton = sum(
            ((cell[axis] >> bit) & 1) << (3 * bit + axis)
            for bit in range(2)
            for axis in range(3)
        )
        mask |= 1 << morton
    return mask & UINT32_MAX, mask >> 32


def check(actual, expected, case):
    if actual != expected:
        raise AssertionError(f"{case}: got {actual!r}, expected {expected!r}")


def main():
    positions = list(product(range(4), repeat=3))
    directions = wrapped = boxes = nonempty = 0
    for octant, position in product(range(8), positions):
        lo = tuple(0 if (octant >> a) & 1 else position[a] for a in range(3))
        hi = tuple(position[a] if (octant >> a) & 1 else 3 for a in range(3))
        expected = cartesian_mask(lo, hi)
        check(direction_reach_mask(octant, position), expected, (octant, position))
        directions += 1
        for offset in ((-4, 4, 8), (4, -8, -4)):
            shifted = tuple(position[a] + offset[a] for a in range(3))
            check(direction_reach_mask(octant, shifted), expected, (octant, shifted))
            wrapped += 1

    for lo, hi in product(positions, repeat=2):
        lower = direction_reach_mask(0, lo)
        upper = direction_reach_mask(7, hi)
        actual = tuple(a & b for a, b in zip(lower, upper))
        expected = cartesian_mask(lo, hi)
        check(actual, expected, ("box", lo, hi))
        boxes += 1
        nonempty += expected != (0, 0)

    print(
        f"PASS: {directions} direction masks, {wrapped} wrapped-position masks, "
        f"and {boxes} bounding masks ({nonempty} nonempty, {boxes - nonempty} empty)."
    )


if __name__ == "__main__":
    main()
