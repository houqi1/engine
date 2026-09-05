"""CPU model of voxel_beam.glsl, checked against independent per-ray slab hits.

Uses Python double precision, not a GPU/GLSL floating-point execution test.
"""

import math
import random
import unittest
from dataclasses import dataclass, field, replace
from functools import lru_cache
from itertools import product

from test_direction_masks import direction_reach_mask


ROUND = 8.0e-6
FAR = 1.0e4
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def add(*vectors):
    return tuple(map(sum, zip(*vectors)))


def scale(vector, factor):
    return tuple(x * factor for x in vector)


def matvec(matrix, vector):
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def length(vector):
    return math.sqrt(sum(x * x for x in vector))


def pixel_vectors(beam, size, projection, view):
    first = tuple(x * 8 for x in beam)
    last = tuple(min(first[a] + 7, size[a] - 1) for a in range(2))
    return [
        matvec(view, tuple(((p[a] + 0.5) / size[a] * 2 - 1) / projection[a]
                           for a in range(2)) + (-1.0,))
        for p in product(*(range(first[a], last[a] + 1) for a in range(2)))
    ]


def beam_vectors(beam, size, projection, view):
    first = tuple(x * 8 for x in beam)
    last = tuple(min(first[a] + 7, size[a] - 1) for a in range(2))
    endpoints = [tuple(((p[a] + 0.5) / size[a] * 2 - 1) / projection[a]
                       for a in range(2)) for p in (first, last)]
    low = tuple(map(min, zip(*endpoints)))
    high = tuple(map(max, zip(*endpoints)))
    center = matvec(view, tuple((a + b) * 0.5 for a, b in zip(low, high)) + (-1.0,))
    dx = scale(tuple(row[0] for row in view), (high[0] - low[0]) * 0.5)
    dy = scale(tuple(row[1] for row in view), (high[1] - low[1]) * 0.5)
    uv_abs = tuple(max(abs(a), abs(b)) for a, b in zip(low, high))
    magnitude = matvec(tuple(tuple(map(abs, row)) for row in view), uv_abs + (1.0,))
    radius = tuple(abs(x) + abs(y) + ROUND * s for x, y, s in zip(dx, dy, magnitude))
    minimum_norm = length(tuple(max(abs(c) - r, 0) for c, r in zip(center, radius))) * (1 - ROUND)
    return center, dx, dy, magnitude, minimum_norm


@dataclass
class VoxelObject:
    cells: frozenset
    translation: tuple = (0.0, 0.0, 8.0)
    matrix: tuple = IDENTITY
    grid: tuple = (8, 8, 8)
    voxel_size: float = 1.0
    enabled: bool = True
    occupancy_available: bool = True
    nested: bool = True
    # Missing/None = INVALID solid; an empty set is an allocated empty brick.
    bricks: dict = field(default_factory=dict)
    empty_material: frozenset = frozenset()


@dataclass
class NarrowBudget:
    remaining: int = 128
    used: int = 0
    exhausted: bool = False

    def take(self, count):
        if self.remaining < count:
            self.exhausted = True
            return False
        self.remaining -= count
        self.used += count
        return True


def object_bounds(obj, camera, vectors):
    center, dx, dy, magnitude, _ = vectors
    inverse_size = 1.0 / obj.voxel_size
    absolute = tuple(tuple(map(abs, row)) for row in obj.matrix)
    origin = scale(add(matvec(obj.matrix, camera), obj.translation), inverse_size)
    direction = scale(matvec(obj.matrix, center), inverse_size)
    x = matvec(obj.matrix, dx)
    y = matvec(obj.matrix, dy)
    direction_pad = scale(matvec(absolute, magnitude), ROUND * inverse_size)
    extent = tuple((abs(a) + abs(b)) * inverse_size + p for a, b, p in zip(x, y, direction_pad))
    low = tuple(d - e for d, e in zip(direction, extent))
    high = add(direction, extent)
    origin_scale = add(matvec(absolute, tuple(map(abs, camera))), tuple(map(abs, obj.translation)))
    padding = tuple(2.0e-4 + ROUND * s * inverse_size for s in origin_scale)
    return origin, padding, low, high


def swept_box(obj, bounds, a, b):
    origin, padding, low, high = bounds
    pad = tuple(p + ROUND * (abs(o) + b * max(abs(l), abs(h)) + n + 1)
                for o, p, l, h, n in zip(origin, padding, low, high, obj.grid))
    box_low = tuple(o + min(a * d, b * d) - p for o, d, p in zip(origin, low, pad))
    box_high = tuple(o + max(a * d, b * d) + p for o, d, p in zip(origin, high, pad))
    return box_low, box_high


def morton(p):
    return sum(((p[a] >> b) & 1) << (3 * b + a) for b in range(3) for a in range(3))


def decode_coarse(bit):
    return ((bit & 1) | ((bit >> 2) & 2),
            ((bit >> 1) & 1) | ((bit >> 3) & 2),
            ((bit >> 2) & 1) | ((bit >> 4) & 2))


def xyz_box(lo, hi):
    # Shader order matters for budget exhaustion: X changes fastest.
    for z in range(lo[2], hi[2] + 1):
        for y in range(lo[1], hi[1] + 1):
            for x in range(lo[0], hi[0] + 1):
                yield x, y, z


@lru_cache(maxsize=1024)
def box_mask(lo, hi):
    lower, upper = direction_reach_mask(0, lo), direction_reach_mask(7, hi)
    return (lower[0] & upper[0]) | ((lower[1] & upper[1]) << 32)


def micro_masks(lo, hi):
    for octant in xyz_box(tuple(x >> 2 for x in lo), tuple(x >> 2 for x in hi)):
        lower = tuple(max(lo[a] - 4 * octant[a], 0) for a in range(3))
        upper = tuple(min(hi[a] - 4 * octant[a], 3) for a in range(3))
        yield octant[0] | (octant[1] << 1) | (octant[2] << 2), box_mask(lower, upper)


def narrow_blocked(obj, overlap, tile, box_low, box_high, budget):
    for _ in range(64):
        if not overlap:
            return False
        if not budget.take(1):
            return True
        bit = (overlap & -overlap).bit_length() - 1
        overlap &= overlap - 1
        coarse = tuple(4 * t + p for t, p in zip(tile, decode_coarse(bit)))
        if coarse in obj.empty_material:
            continue
        micros = obj.bricks.get(coarse)
        if micros is None:
            return True
        bits = sum(1 << morton(p) for p in micros)
        lo = tuple(max(0, min(7, math.floor((x - c) * 8 - 1.0e-4)))
                   for x, c in zip(box_low, coarse))
        hi = tuple(max(0, min(7, math.floor((x - c) * 8 + 1.0e-4)))
                   for x, c in zip(box_high, coarse))
        for octant, mask in micro_masks(lo, hi):
            if not budget.take(2):
                return True
            if ((bits >> (64 * octant)) & mask) != 0:
                return True
    return False


def tile_words(cells):
    tiles = {}
    for cell in cells:
        tile = tuple(c >> 2 for c in cell)
        bit = morton(tuple(c & 3 for c in cell))
        tiles[tile] = tiles.get(tile, 0) | (1 << bit)
    return tiles


def conservative_beam(objects, beam=(4, 4), size=(64, 64), projection=(1.0, 1.0),
                      view=IDENTITY, camera=(0.0, 0.0, 0.0), intervals=64,
                      tile_budget=128, tile_limit=32, refinements=None, stage=0,
                      narrow_budget=None):
    vectors = beam_vectors(beam, size, projection, view)
    minimum_norm = vectors[-1]
    if not minimum_norm > 1.0e-8:
        return 0.0
    safe = FAR / minimum_norm
    budget = NarrowBudget() if narrow_budget is None else narrow_budget
    for obj in objects:
        if not obj.enabled or obj.voxel_size <= 0 or 0 in obj.grid:
            continue
        if len(set(obj.grid)) != 1:
            return 0.0
        bounds = object_bounds(obj, camera, vectors)
        origin, padding, low, high = bounds
        enter, exit_depth, misses = 0.0, safe, False
        for o, p, lo, hi, n in zip(origin, padding, low, high, obj.grid):
            lower, upper = -o - p, n - o + p
            if hi > 0:
                enter = max(enter, lower / hi)
            elif hi < 0:
                exit_depth = min(exit_depth, lower / hi)
            elif lower > 0:
                misses = True
            if lo > 0:
                exit_depth = min(exit_depth, upper / lo)
            elif lo < 0:
                enter = max(enter, upper / lo)
            elif upper < 0:
                misses = True
        enter = max(0, enter - ROUND * (1 + abs(enter)))
        exit_depth = min(safe, exit_depth + ROUND * (1 + abs(exit_depth)))
        if misses or enter > exit_depth:
            continue
        if enter == exit_depth or not obj.occupancy_available:
            safe = min(safe, enter)
            continue
        speed = max(max(map(abs, low)), max(map(abs, high)))
        if speed == 0:
            return 0.0
        step = 4 / speed
        allow_micro = obj.nested and stage < 3
        refine = (5 if allow_micro else 2) if refinements is None else refinements
        a, reads_left = enter, tile_budget
        tiles = tile_words(obj.cells)
        for _ in range(intervals):
            if a >= exit_depth:
                break
            b = min(exit_depth, a + step)
            if b <= a:
                break
            box_low, box_high = swept_box(obj, bounds, a, b)
            if any(h < 0 or l > n for l, h, n in zip(box_low, box_high, obj.grid)):
                a = b
                continue
            cell_low = tuple(max(0, min(math.floor(x), n - 1)) for x, n in zip(box_low, obj.grid))
            cell_high = tuple(max(0, min(math.floor(x), n - 1)) for x, n in zip(box_high, obj.grid))
            tile_low = tuple(x >> 2 for x in cell_low)
            tile_high = tuple(x >> 2 for x in cell_high)
            count = math.prod(h - l + 1 for l, h in zip(tile_low, tile_high))
            if count > tile_limit or count > reads_left:
                break
            reads_left -= count
            blocked = False
            budget.exhausted = False
            for tile in xyz_box(tile_low, tile_high):
                bits = tiles.get(tile, 0)
                if bits:
                    lo = tuple(max(c - 4 * t, 0) for c, t in zip(cell_low, tile))
                    hi = tuple(min(c - 4 * t, 3) for c, t in zip(cell_high, tile))
                    overlap = bits & box_mask(lo, hi)
                    if overlap and (not allow_micro or
                                    narrow_blocked(obj, overlap, tile, box_low, box_high, budget)):
                        blocked = True
                        break
            if blocked:
                if budget.exhausted or refine == 0:
                    break
                step *= 0.5
                refine -= 1
            else:
                a = b
        if a < exit_depth:
            safe = min(safe, a)
        if safe <= 0:
            return 0.0
    world_t = min(FAR, safe * minimum_norm)
    return max(0, world_t - ROUND * (1 + world_t))


def ray_box(origin, direction, lo, hi):
    enter, exit_depth = 0.0, math.inf
    for o, d, low, high in zip(origin, direction, lo, hi):
        if d == 0:
            if not low <= o <= high:
                return math.inf
        else:
            times = ((low - o) / d, (high - o) / d)
            enter, exit_depth = max(enter, min(times)), min(exit_depth, max(times))
    return enter if enter <= exit_depth else math.inf


def exact_hits(objects, beam=(4, 4), size=(64, 64), projection=(1.0, 1.0),
               view=IDENTITY, camera=(0.0, 0.0, 0.0), stage=0):
    hits = []
    for ray in pixel_vectors(beam, size, projection, view):
        ray = scale(ray, 1 / length(ray))
        hit = math.inf
        for obj in objects:
            if not obj.enabled or obj.voxel_size <= 0 or 0 in obj.grid:
                continue
            origin = scale(add(matvec(obj.matrix, camera), obj.translation), 1 / obj.voxel_size)
            direction = scale(matvec(obj.matrix, ray), 1 / obj.voxel_size)
            for cell in obj.cells:
                if cell in obj.empty_material:
                    continue
                micros = obj.bricks.get(cell) if obj.nested and stage < 3 else None
                if micros is None:
                    hit = min(hit, ray_box(origin, direction, cell, tuple(c + 1 for c in cell)))
                else:
                    # Independent geometry reference: no Morton masks or interval sweep.
                    for micro in micros:
                        lo = tuple(c + m / 8 for c, m in zip(cell, micro))
                        hit = min(hit, ray_box(origin, direction, lo, tuple(x + 0.125 for x in lo)))
        hits.append(hit)
    return hits


class BeamTests(unittest.TestCase):
    def assert_safe(self, objects, **kwargs):
        actual = conservative_beam(objects, **kwargs)
        ray_args = {k: v for k, v in kwargs.items()
                    if k not in ("intervals", "tile_budget", "tile_limit", "refinements", "narrow_budget")}
        expected = min(exact_hits(objects, **ray_args))
        self.assertTrue(math.isfinite(actual))
        self.assertGreaterEqual(actual, 0)
        self.assertLessEqual(actual, min(FAR, expected) + 1.0e-9)
        return actual

    def test_all_single_cells_and_reversed_axes(self):
        poses = (
            (IDENTITY, (0.0, 0.0, 12.0)),
            (IDENTITY, (2.5, 2.5, 4.0)),
            (((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)), (8.0, 8.0, 12.0)),
            (((0.0, 0.0, -2.0), (0.0, 0.5, 0.0), (-1.0, 0.0, 0.0)), (-4.0, 2.0, 5.0)),
        )
        for matrix, translation in poses:
            for cell in product(range(8), repeat=3):
                self.assert_safe([VoxelObject(frozenset([cell]), translation, matrix)])

    def test_interior_hit_missed_by_five_samples(self):
        objects = [VoxelObject(frozenset([(1, 2, 15)]), (0.0, 0.0, 32.0), grid=(64, 64, 64))]
        hits = exact_hits(objects)
        self.assertTrue(math.isfinite(min(hits)))
        self.assertTrue(all(math.isinf(hits[x * 8 + y]) for x, y in
                            ((0, 0), (7, 0), (0, 7), (7, 7), (4, 4))))
        self.assertGreater(self.assert_safe(objects), 0)

    def test_budget_fallbacks_and_empty_completion(self):
        obj = VoxelObject(frozenset([(1, 1, 2)]), (0.0, 0.0, 12.0))
        for limits in ({"intervals": 0}, {"intervals": 1}, {"tile_budget": 0},
                       {"tile_budget": 1}, {"tile_limit": 1}, {"refinements": 0}):
            with self.subTest(limits=limits):
                self.assertLess(self.assert_safe([obj], **limits), FAR / 2)
        empty = VoxelObject(frozenset(), (0.0, 0.0, 12.0))
        self.assertLess(conservative_beam([empty], intervals=0), FAR / 2)
        self.assertGreater(self.assert_safe([empty]), FAR - 1)

    def test_missing_occupancy_disabled_behind_and_camera_inside(self):
        solid = frozenset(product(range(8), repeat=3))
        missing = VoxelObject(solid, occupancy_available=False)
        self.assert_safe([missing])
        self.assertEqual(self.assert_safe([VoxelObject(solid, (4.0, 4.0, 4.0))]), 0)
        behind = VoxelObject(solid, (0.0, 0.0, -1.0))
        disabled = VoxelObject(solid, (4.0, 4.0, 4.0), enabled=False)
        self.assertGreater(self.assert_safe([behind, disabled]), FAR - 1)

    def test_axis_parallel_partial_tiles_and_negative_projection(self):
        obj = VoxelObject(frozenset([(0, 0, 0)]), (0.0, 0.0, 12.0), grid=(6, 6, 6))
        self.assert_safe([obj], beam=(0, 0), size=(1, 1))
        self.assert_safe([obj], beam=(8, 8), size=(65, 67), projection=(-1.0, -1.0))
        self.assert_safe([obj], beam=(8, 8), size=(65, 67), projection=(1.0, -1.0))
        bad_layout = VoxelObject(obj.cells, grid=(6, 8, 6))
        self.assertEqual(conservative_beam([bad_layout]), 0)

    def test_random_bounds_and_multiple_objects(self):
        rng = random.Random(7441)
        for _ in range(40):
            angle = rng.uniform(-math.pi, math.pi)
            c, s = math.cos(angle), math.sin(angle)
            view = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
            beam = (rng.randrange(7), rng.randrange(7))
            projection = (rng.choice((-1.0, 1.0)), rng.choice((-1.3, 1.3)))
            args = dict(beam=beam, size=(53, 55), projection=projection, view=view)
            objects = [VoxelObject(frozenset(tuple(rng.randrange(8) for _ in range(3))
                                             for _ in range(6)),
                                   tuple(rng.uniform(-5, 13) for _ in range(3))) for _ in range(2)]
            vectors = beam_vectors(**args)
            rays = pixel_vectors(**args)
            self.assertLessEqual(vectors[-1], min(map(length, rays)))
            bounds = object_bounds(objects[0], (0.0, 0.0, 0.0), vectors)
            a = rng.uniform(0, 20)
            b = a + rng.uniform(0, 8)
            lo, hi = swept_box(objects[0], bounds, a, b)
            for ray, depth in product(rays, (a, (a + b) * 0.5, b)):
                point = add(bounds[0], scale(matvec(objects[0].matrix, ray), depth / objects[0].voxel_size))
                self.assertTrue(all(l <= p <= h for l, p, h in zip(lo, point, hi)))
            self.assert_safe(objects, **args)

    def test_nonrigid_camera_and_large_translation(self):
        view = ((-1.5, 0.2, 0.0), (0.0, 0.5, 0.0), (0.0, 0.0, -2.0))
        camera = (100000.0, -100000.0, 100000.0)
        translation = (-99996.0, 100004.0, -100004.0)
        obj = VoxelObject(frozenset([(3, 4, 2), (2, 4, 5)]), translation)
        self.assert_safe([obj], view=view, camera=camera)

    def test_64_coarse_decodes_and_46656_micro_boxes(self):
        for bit in range(64):
            self.assertEqual(morton(decode_coarse(bit)), bit)
        points = {p: 1 << morton(p) for p in product(range(8), repeat=3)}
        intervals = [(lo, hi) for lo in range(8) for hi in range(lo, 8)]
        count = 0
        for axes in product(intervals, repeat=3):
            lo, hi = tuple(a[0] for a in axes), tuple(a[1] for a in axes)
            actual = sum(mask << (64 * octant) for octant, mask in micro_masks(lo, hi))
            expected = sum(points[p] for p in product(*(range(l, h + 1) for l, h in axes)))
            self.assertEqual(actual, expected, (lo, hi))
            count += 1
        self.assertEqual(count, 46656)

    def test_micro_boundaries_and_read_guards(self):
        cell = (0, 0, 0)
        empty = VoxelObject(frozenset([cell]), bricks={cell: frozenset()})
        for limit in range(18):
            budget = NarrowBudget(limit)
            blocked = narrow_blocked(empty, 1, cell, cell, (1, 1, 1), budget)
            self.assertEqual(blocked, limit < 17, limit)  # One texel plus eight word pairs.
            self.assertEqual(budget.exhausted, blocked)
            self.assertLessEqual(budget.used, limit)
        for micro in product((3, 4), repeat=3):
            obj = replace(empty, bricks={cell: frozenset([micro])})
            self.assertTrue(narrow_blocked(obj, 1, cell, (0.5,) * 3, (0.5,) * 3, NarrowBudget()))
        for coarse, micro in (((0, 0, 0), (7, 0, 0)), ((1, 0, 0), (0, 0, 0))):
            obj = VoxelObject(frozenset([coarse]), bricks={coarse: frozenset([micro])})
            self.assertTrue(narrow_blocked(obj, 1 << morton(coarse), cell,
                                           (1, 0.01, 0.01), (1, 0.02, 0.02), NarrowBudget()))
        material_zero = replace(empty, empty_material=frozenset([cell]))
        budget = NarrowBudget(1)
        self.assertFalse(narrow_blocked(material_zero, 1, cell, cell, (1, 1, 1), budget))
        self.assertEqual(budget.used, 1)
        self.assertFalse(narrow_blocked(empty, 0, cell, cell, (1, 1, 1), NarrowBudget(0)))
        self.assertTrue(narrow_blocked(replace(empty, bricks={}), 1, cell, cell, (1, 1, 1), NarrowBudget()))

    def test_512_single_micro_scenes_and_front_prefix(self):
        cell = (0, 0, 0)
        obj = VoxelObject(frozenset([cell]), (0.0, 0.0, 2.0), grid=(4, 4, 4))
        for micro in product(range(8), repeat=3):
            self.assert_safe([replace(obj, bricks={cell: frozenset([micro])})])
        obj = replace(obj, bricks={cell: frozenset([cell])})
        args = dict(beam=(160, 90), size=(2560, 1440))
        coarse = self.assert_safe([replace(obj, nested=False)], **args)
        fine = self.assert_safe([obj], **args)
        self.assertGreater(fine, coarse + 0.5)
        self.assertGreater(fine, self.assert_safe([obj], refinements=2, **args))
        for limit in (0, 1, 2, 3, 16, 32, 128):
            budget = NarrowBudget(limit)
            self.assert_safe([obj], narrow_budget=budget, **args)
            self.assertLessEqual(budget.used, limit)
        for stage in (0, 2, 3):
            budget = NarrowBudget()
            value = self.assert_safe([obj], stage=stage, narrow_budget=budget, **args)
            self.assertEqual(value, coarse if stage == 3 else fine)
            self.assertEqual(budget.used == 0, stage == 3)
        budget = NarrowBudget()
        self.assert_safe([replace(obj, nested=False)], narrow_budget=budget, **args)
        self.assertEqual(budget.used, 0)

    def test_global_budget_and_last_certified_prefix(self):
        cell = (0, 0, 0)
        empty = VoxelObject(frozenset([cell]), (0.4, 0.4, 2.0), grid=(1, 1, 1),
                            bricks={cell: frozenset()})
        self.assertEqual(NarrowBudget().remaining, 128)
        self.assertGreater(self.assert_safe([empty], narrow_budget=NarrowBudget(17)), FAR - 1)
        budget = NarrowBudget(17)
        value = self.assert_safe([empty, empty], narrow_budget=budget)
        self.assertEqual(budget.used, 17)
        self.assertTrue(budget.exhausted)
        self.assertTrue(0 < value < 2)  # Resetting the budget per object would report FAR.

        cells = frozenset((0, 0, z) for z in (1, 6, 11))
        obj = VoxelObject(cells, (0.4, 0.4, 12.0), grid=(12, 12, 12),
                          bricks={(0, 0, z): frozenset() for z in (6, 11)})
        budget = NarrowBudget(5)
        value = self.assert_safe([obj], beam=(160, 90), size=(2560, 1440), narrow_budget=budget)
        self.assertEqual(budget.used, 5)
        self.assertTrue(budget.exhausted)
        self.assertTrue(3.9 < value < 4.1)  # One whole interval was cleared before exhaustion.

    def test_32_multibrick_scenes(self):
        rng = random.Random(1559)
        poses = (IDENTITY, ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
                 ((0.0, 0.0, -2.0), (0.0, 0.5, 0.0), (-1.0, 0.0, 0.0)))
        for _ in range(32):
            objects = []
            for _ in range(2):
                bricks = {tuple(rng.randrange(4) for _ in range(3)):
                          frozenset(tuple(rng.randrange(8) for _ in range(3)) for _ in range(5))
                          for _ in range(4)}
                objects.append(VoxelObject(frozenset(bricks),
                                           tuple(rng.uniform(0, 6) for _ in range(3)),
                                           rng.choice(poses), (4, 4, 4), bricks=bricks))
            budget = NarrowBudget(rng.choice((0, 1, 2, 3, 16, 32, 128)))
            limit = budget.remaining
            self.assert_safe(objects, narrow_budget=budget)
            self.assertLessEqual(budget.used, limit)


if __name__ == "__main__":
    unittest.main(verbosity=2)
