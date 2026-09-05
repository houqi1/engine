"""Small traceFine CPU regression model versus independent 2^3 cell AABBs.

Standard library only. Uses Python doubles, not GPU/GLSL FP32 execution.
Rays are nonzero and start inside the microcell, as in the parent traversal.
"""

import math
import random
import unittest
from itertools import product
from pathlib import Path


FLT_MAX = 3.4028235e38
CELLS = tuple(product(range(2), repeat=3))


def trace_fine(bits, origin, direction, enter_mask=(True, False, False)):
    origin = tuple(max(0.0001, min(1.9999, x)) for x in origin)
    cell = tuple(math.floor(x) for x in origin)
    step = tuple(-1 if d < 0 else 1 for d in direction)
    delta = tuple(min(abs(1 / d), FLT_MAX) if d else FLT_MAX for d in direction)
    side = tuple((s * (c - o) + (s * 0.5 + 0.5)) * dt if d else FLT_MAX
                 for s, c, o, dt, d in zip(step, cell, origin, delta, direction))
    mask, hit_t = enter_mask, 0.0
    # Initial occupancy check, then at most eight steps, matching traceFine.
    for i in range(9):
        if any(c < 0 or c >= 2 for c in cell):
            return None
        if bits & (1 << (cell[1] * 4 + cell[2] * 2 + cell[0])):
            uvw = tuple(max(0.0, min(1.0, o + d * hit_t - c))
                        for o, d, c in zip(origin, direction, cell))
            return cell, hit_t, mask, uvw
        if i == 8:
            break
        hit_t = min(side)
        mask = tuple(s <= min(side[(a + 1) % 3], side[(a + 2) % 3])
                     for a, s in enumerate(side))
        side = tuple(s + m * dt for s, m, dt in zip(side, mask, delta))
        assert all(math.isfinite(s) for s in side)
        cell = tuple(c + m * s for c, m, s in zip(cell, mask, step))
    return None


def brute_aabb(occupied, origin, direction):
    best = None
    for cell in occupied:
        enter, exit_t = 0.0, math.inf
        for low, o, d in zip(cell, origin, direction):
            if d == 0:
                if not low <= o < low + 1:
                    break
            else:
                a, b = (low - o) / d, (low + 1 - o) / d
                enter, exit_t = max(enter, min(a, b)), min(exit_t, max(a, b))
        else:
            # DDA advances all tied axes: point-only edge/corner contacts are not hits.
            if enter < exit_t and (best is None or enter < best[1]):
                best = cell, enter
    return best


class TraversalTests(unittest.TestCase):
    def assert_matches(self, occupied, origin, direction):
        bits = sum(1 << (y * 4 + z * 2 + x) for x, y, z in occupied)
        actual = trace_fine(bits, origin, direction)
        expected = brute_aabb(occupied, origin, direction)
        case = occupied, origin, direction
        if expected is None:
            self.assertIsNone(actual, case)
        else:
            self.assertIsNotNone(actual, case)
            self.assertEqual(actual[0], expected[0], case)
            self.assertTrue(math.isfinite(actual[1]), case)
            self.assertAlmostEqual(actual[1], expected[1], places=12, msg=case)
            for uv, o, d, c in zip(actual[3], origin, direction, expected[0]):
                self.assertTrue(math.isfinite(uv), case)
                self.assertAlmostEqual(uv, o + d * expected[1] - c, places=12, msg=case)
        return actual

    def test_known_tie_is_075_not_old_15(self):
        hit = self.assert_matches({(1, 1, 0)}, (0.25,) * 3, (1.0, 1.0, 0.25))
        self.assertEqual(hit[1], 0.75)
        self.assertEqual(hit[2], (True, True, False))
        self.assertEqual(hit[3], (0.0, 0.0, 0.4375))
        # Old dot(sideDist - deltaDist, mask) after stepping sums both tied times.
        old_t = sum((s - d) * m for s, d, m in
                    zip((1.75, 1.75, 3.0), (1.0, 1.0, 4.0), hit[2]))
        self.assertEqual(old_t, 1.5)
        hit = self.assert_matches({(1, 1, 1)}, (0.25,) * 3, (1.0,) * 3)
        self.assertEqual(hit[1], 0.75)
        self.assertEqual(hit[2], (True,) * 3)

    def test_axis_parallel_finite_hits_and_misses(self):
        for axis, sign, zero in product(range(3), (-1, 1), (0.0, -0.0)):
            origin = tuple((0.25 if sign > 0 else 1.75) if a == axis else 0.25
                           for a in range(3))
            direction = tuple(float(sign) if a == axis else zero for a in range(3))
            target = tuple(int(sign > 0) if a == axis else 0 for a in range(3))
            hit = self.assert_matches({target}, origin, direction)
            self.assertEqual(hit[1], 0.75)
            self.assertEqual(hit[2], tuple(a == axis for a in range(3)))
            off_ray = tuple(1 - c if a == (axis + 1) % 3 else c
                            for a, c in enumerate(target))
            self.assert_matches({off_ray}, origin, direction)

    def test_initial_hit_preserves_entry_mask(self):
        mask = (False, True, False)
        hit = trace_fine(1, (0.25,) * 3, (0.0, -1.0, 0.0), mask)
        self.assertEqual(hit, ((0, 0, 0), 0.0, mask, (0.25,) * 3))

    def test_all_256_patterns_8_origins_26_directions(self):
        origins = tuple(product((0.25, 1.75), repeat=3))
        directions = tuple(d for d in product((-1.0, 0.0, 1.0), repeat=3) if any(d))
        for pattern in range(256):
            occupied = {c for i, c in enumerate(CELLS) if pattern & (1 << i)}
            for origin, direction in product(origins, directions):
                self.assert_matches(occupied, origin, direction)

    def test_seeded_unequal_slopes(self):
        rng = random.Random(7541)
        for _ in range(512):
            occupied = {c for c in CELLS if rng.getrandbits(1)}
            origin = tuple(rng.uniform(0.001, 1.999) for _ in range(3))
            direction = tuple(rng.uniform(-1, 1) for _ in range(3))
            self.assert_matches(occupied, origin, direction)

    def test_shader_keeps_finite_inverse_and_min_before_step(self):
        shader = (Path(__file__).resolve().parents[1] / "shaders" / "voxel_dda.comp").read_text()
        fine = shader.split("bool traceFine(", 1)[1].split("bool traceMicro(", 1)[0]
        self.assertIn("abs(clamp(1.0 / rd, vec3(-FLT_MAX), vec3(FLT_MAX)))", fine)
        self.assertIn("sideDist = mix(sideDist, vec3(FLT_MAX), equal(rd, vec3(0.0)))", fine)
        self.assertLess(fine.index("float tHit = min(sideDist.x, min(sideDist.y, sideDist.z))"),
                        fine.index("sideDist += vec3(mask) * deltaDist"))
        self.assertIn("outT = tHit;", fine)
        self.assertNotIn("dot(sideDist - deltaDist", fine)


if __name__ == "__main__":
    unittest.main(verbosity=2)
