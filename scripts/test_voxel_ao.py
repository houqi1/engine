"""CPU model of voxel_ao.glsl versus eight independent fine occupancy queries.

Uses only the standard library; does not compile or execute GLSL.
"""

import random
from itertools import permutations, product


INVALID_PAGE = 0xFFFFFFFF
MICRO_WORDS = 16
AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


def add(*vectors):
    return tuple(map(sum, zip(*vectors)))


def neg(vector):
    return tuple(-x for x in vector)


def morton(p):
    return sum(((p[a] >> b) & 1) << (3 * b + a)
               for b in range(3) for a in range(3))


def fine_bit(p):
    return (p[1] & 1) * 4 + (p[2] & 1) * 2 + (p[0] & 1)


def load_fine_ao_byte(grid, cells, pages, micro_pos):
    # Previous helper, retained as an independent uncached oracle.
    coarse = tuple(x >> 3 for x in micro_pos)
    if any(x < 0 or x >= n for x, n in zip(coarse, grid)):
        return 0
    material, page = cells[coarse]
    if material == 0:
        return 0
    if page == INVALID_PAGE:
        return 255
    bit = morton(tuple(x & 7 for x in micro_pos))
    # The shader relies on the mutator invariant, without reading micro occupancy.
    packed = pages[page][MICRO_WORDS + (bit >> 2)]
    return (packed >> ((bit & 3) * 8)) & 255


def load_fine_ao_byte_known_hit(grid, cells, pages, micro_pos, hit_coarse, hit_page):
    # Current loadFineAoByte: hit_coarse is an in-grid, known-solid coarse cell.
    coarse = tuple(x >> 3 for x in micro_pos)
    page = hit_page
    if coarse != hit_coarse:
        if any(x < 0 or x >= n for x, n in zip(coarse, grid)):
            return 0
        material, page = cells[coarse]
        if material == 0:
            return 0
    if page == INVALID_PAGE:
        return 255
    bit = morton(tuple(x & 7 for x in micro_pos))
    packed = pages[page][MICRO_WORDS + (bit >> 2)]
    return (packed >> ((bit & 3) * 8)) & 255


def solid_fine_global(grid, cells, pages, p):
    # Reference follows solidFineGlobal/getFineVoxel, without the byte loader.
    coarse = tuple(x // 16 for x in p)
    if any(not 0 <= x < n for x, n in zip(coarse, grid)):
        return False
    material, page = cells[coarse]
    if material == 0:
        return False
    if page == INVALID_PAGE:
        return True
    micro = tuple(x // 2 % 8 for x in p)
    bit = morton(micro)
    if not pages[page][bit // 32] & (1 << (bit % 32)):
        return False
    fine = tuple(x % 2 for x in p)
    shift = (bit % 4) * 8 + fine[1] * 4 + fine[2] * 2 + fine[0]
    return bool(pages[page][MICRO_WORDS + bit // 4] & (1 << shift))


def ao_values(side, corner):
    return tuple(1.0 - (side[i] + side[(i + 1) % 4]
                       + max(corner[i], side[i] * side[(i + 1) % 4])) / 3.0
                 for i in range(4))


def stencil(p, d1, d2):
    return tuple(add(p, offset) for offset in (
        d1, d2, neg(d1), neg(d2), add(d1, d2),
        add(neg(d1), d2), add(neg(d1), neg(d2)), add(d1, neg(d2))))


def voxel_ao(p, d1, d2, query):
    values = tuple(float(query(q)) for q in stencil(p, d1, d2))
    side, corner = values[:4], values[4:]
    return side, corner, ao_values(side, corner)


def fine_voxel_ao(p, d1, d2, load_byte):
    # Mirror the shader's four lanes and fixed component selections.
    micros = tuple(tuple(x >> 1 for x in q) for q in (
        add(p, d1, d2), add(p, neg(d1), d2),
        add(p, neg(d1), neg(d2)), add(p, d1, neg(d2))))
    assert len(set(micros)) == 4
    x, y, z, w = (load_byte(q) for q in micros)
    center_micro = tuple(x >> 1 for x in p)
    near1 = tuple(x >> 1 for x in add(p, d1)) == center_micro
    near2 = tuple(x >> 1 for x in add(p, d2)) == center_micro
    side_bytes = (x if near2 else w, x if near1 else y,
                  y if near2 else z, w if near1 else z)
    bit1, bit2 = fine_bit(add(p, d1)), fine_bit(add(p, d2))
    side = tuple(float((byte >> bit) & 1)
                 for byte, bit in zip(side_bytes, (bit1, bit2, bit1, bit2)))
    corner = tuple(float((byte >> fine_bit(add(p, d1, d2))) & 1)
                   for byte in (x, y, z, w))
    return side, corner, ao_values(side, corner)


def check(actual, expected, case):
    if actual != expected:
        raise AssertionError(f"{case}: got {actual!r}, expected {expected!r}")


def write_fine_byte(page, bit, value):
    index, shift = MICRO_WORDS + bit // 4, (bit % 4) * 8
    page[index] = (page[index] & ~(255 << shift)) | (value << shift)


def make_page(fine_bytes):
    page = [0] * (MICRO_WORDS + 128)
    for bit, value in enumerate(fine_bytes):
        page[bit // 32] |= int(value != 0) << (bit % 32)
        write_fine_byte(page, bit, value)
    return page


def mutate_micro(page, bit, solid):
    # Allocated-page part of VoxelScene::setMicroCpu, including its no-op rule.
    word, mask = bit // 32, 1 << (bit % 32)
    if bool(page[word] & mask) == solid:
        return False
    page[word] = page[word] | mask if solid else page[word] & ~mask
    write_fine_byte(page, bit, 255 if solid else 0)
    return True


def mutate_fine(page, bit, fine, solid):
    # Occupancy part of VoxelScene::setFineCpu; colors/allocation are not modeled.
    word, mask = bit // 32, 1 << (bit % 32)
    micro_was = bool(page[word] & mask)
    if not micro_was and not solid:
        return False
    byte = (page[MICRO_WORDS + bit // 4] >> ((bit % 4) * 8)) & 255
    fmask = 1 << fine
    if bool(byte & fmask) == solid:
        return False
    byte = byte | fmask if solid else byte & ~fmask
    write_fine_byte(page, bit, byte)
    if solid and not micro_was:
        page[word] |= mask
    elif byte == 0 and micro_was:
        page[word] &= ~mask
    return True


def test_mutation_invariant():
    rng = random.Random(0xED17)
    base = make_page(rng.randrange(1, 256) if rng.getrandbits(1) else 0 for _ in range(512))

    def verify(after, before, bit, value, case):
        word, mask = bit // 32, 1 << (bit % 32)
        index, shift = MICRO_WORDS + bit // 4, (bit % 4) * 8
        expected = before.copy()
        expected[word] = (before[word] & ~mask) | (int(value != 0) << (bit % 32))
        expected[index] = (before[index] & ~(255 << shift)) | (value << shift)
        check(after, expected, case)  # Other bytes, occupancy bits and words must survive.
        byte = (after[index] >> shift) & 255
        check(bool(after[word] & mask), byte != 0, ("invariant", case))

    fine_edits = micro_edits = 0
    # All byte states and edits at all 32 occupancy bit offsets, across a word
    # boundary. This also exercises all four fine-byte lanes and no-op writes.
    for bit, old in product(range(240, 272), range(256)):
        before = base.copy()
        write_fine_byte(before, bit, old)
        before[bit // 32] = ((before[bit // 32] & ~(1 << (bit % 32))) |
                            (int(old != 0) << (bit % 32)))
        for fine, solid in product(range(8), (False, True)):
            page = before.copy()
            changed = mutate_fine(page, bit, fine, solid)
            expected = old | (1 << fine) if solid else old & ~(1 << fine)
            case = ("fine edit", bit, old, fine, solid)
            check(changed, expected != old, case)
            verify(page, before, bit, expected, case)
            fine_edits += 1
        for solid in (False, True):
            page = before.copy()
            changed = mutate_micro(page, bit, solid)
            expected = old if bool(old) == solid else (255 if solid else 0)
            case = ("micro edit", bit, old, solid)
            check(changed, expected != old, case)
            verify(page, before, bit, expected, case)
            micro_edits += 1

    # Deliberately invalid data must expose why the unchecked AO loader needs the
    # invariant. This catches accidentally restoring the old micro check in this model.
    broken = make_page([0] * 512)
    write_fine_byte(broken, 0, 1)
    args = ((1, 1, 1), {(0, 0, 0): (7, 0)}, {0: broken}, (0, 0, 0))
    check(load_fine_ao_byte(*args), 1, "unchecked loader negative control")
    check(load_fine_ao_byte_known_hit(*args, (0, 0, 0), 0), 1, "known-hit negative control")
    check(solid_fine_global(*args), False, "micro-gated reference negative control")
    return fine_edits, micro_edits


class ReadLog:
    def __init__(self, data):
        self.data = data
        self.reads = []

    def __getitem__(self, key):
        self.reads.append(key)
        return self.data[key]


def test_known_hit(grid, cells, pages, positions, bases):
    tracked_cells, tracked_pages = ReadLog(cells), ReadLog(pages)
    routes = set()

    def check_reads(micros, hit_coarse):
        expected_cells, expected_pages = [], []
        for micro in micros:
            coarse = tuple(x // 8 for x in micro)
            if any(not 0 <= x < n for x, n in zip(coarse, grid)):
                routes.add("outside")
                continue
            material, page = cells[coarse]
            same = coarse == hit_coarse
            if not same:
                expected_cells.append(coarse)
            kind = "empty" if material == 0 else ("virtual" if page == INVALID_PAGE else "allocated")
            routes.add(("same" if same else "cross", kind))
            if material != 0 and page != INVALID_PAGE:
                expected_pages.append(page)
        check(tracked_cells.reads, expected_cells, ("coarse reads", hit_coarse, micros))
        check(tracked_pages.reads, expected_pages, ("page reads", hit_coarse, micros))

    # Every microcell and exterior shell against every valid known hit brick,
    # plus negative whole-coarse boundaries that distinguish floor from truncation.
    micros = list(product(*(range(-1, n * 8 + 1) for n in grid)))
    micros += [tuple(value * x for x in axis) for axis in AXES for value in (-9, -8, -1)]
    byte_checks = 0
    for hit_coarse, (material, hit_page) in cells.items():
        if material == 0:
            continue
        if hit_page != INVALID_PAGE:
            check(any(pages[hit_page][MICRO_WORDS:]), True, ("nonempty hit brick", hit_coarse))
        for micro in micros:
            tracked_cells.reads.clear()
            tracked_pages.reads.clear()
            check(load_fine_ao_byte_known_hit(grid, tracked_cells, tracked_pages, micro, hit_coarse, hit_page),
                  load_fine_ao_byte(grid, cells, pages, micro), ("known byte", hit_coarse, micro))
            check_reads((micro,), hit_coarse)
            byte_checks += 1

    byte_queries, fine_queries = [], []

    def load_byte(q):
        byte_queries.append(q)
        return load_fine_ao_byte_known_hit(grid, tracked_cells, tracked_pages, q, hit_coarse, hit_page)

    def query(q):
        fine_queries.append(q)
        return solid_fine_global(grid, cells, pages, q)

    ao_checks, normals, hit_kinds = 0, set(), set()
    for solid_pos in positions:
        if not solid_fine_global(grid, cells, pages, solid_pos):
            continue
        hit_coarse = tuple(x // 16 for x in solid_pos)
        _, hit_page = cells[hit_coarse]
        for d1, d2 in bases:
            axis = AXES[next(i for i in range(3) if d1[i] == 0 and d2[i] == 0)]
            for sign in (-1, 1):
                normal = tuple(sign * x for x in axis)
                ao_pos = add(solid_pos, normal)
                if solid_fine_global(grid, cells, pages, ao_pos):
                    continue  # Only actual exposed faces, not arbitrary cached hit identities.
                tracked_cells.reads.clear()
                tracked_pages.reads.clear()
                byte_queries.clear()
                fine_queries.clear()
                check(fine_voxel_ao(ao_pos, d1, d2, load_byte), voxel_ao(ao_pos, d1, d2, query),
                      ("known-hit AO", solid_pos, normal, d1, d2))
                check(len(byte_queries), 4, "four byte gathers")
                check(len(set(byte_queries)), 4, "four distinct microcells")
                check(tuple(fine_queries), stencil(ao_pos, d1, d2), "eight independent fine queries")
                check_reads(byte_queries, hit_coarse)
                normals.add(normal)
                hit_kinds.add("virtual" if hit_page == INVALID_PAGE else "allocated")
                ao_checks += 1
    check(routes, {"outside", ("same", "virtual"), ("same", "allocated"),
                   ("cross", "virtual"), ("cross", "allocated"), ("cross", "empty")}, "cache routes")
    check(normals, {tuple(sign * x for x in a) for a in AXES for sign in (-1, 1)}, "six face normals")
    check(hit_kinds, {"virtual", "allocated"}, "real hit kinds")
    return byte_checks, ao_checks


def main():
    fine_edits, micro_edits = test_mutation_invariant()
    rng = random.Random(0xA04)
    bases = [(tuple(s1 * x for x in a), tuple(s2 * x for x in b))
             for a, b in permutations(AXES, 2) for s1, s2 in product((-1, 1), repeat=2)]
    stencils = 0
    for parity, (d1, d2) in product(product(range(2), repeat=3), bases):
        p = add((14, 16, 30), parity)
        positions = stencil(p, d1, d2)
        for pattern in range(256):
            neighbors = {q: (pattern >> i) & 1 for i, q in enumerate(positions)}
            fine_bytes = {tuple(x >> 1 for x in q): rng.getrandbits(8) for q in positions}
            for q, occupied in neighbors.items():
                micro = tuple(x // 2 for x in q)
                bit = (q[1] % 2) * 4 + (q[2] % 2) * 2 + q[0] % 2
                fine_bytes[micro] = (fine_bytes[micro] & ~(1 << bit)) | (occupied << bit)
            check(fine_voxel_ao(p, d1, d2, fine_bytes.__getitem__),
                  voxel_ao(p, d1, d2, neighbors.__getitem__), (parity, d1, d2, pattern))
            stencils += 1

    # Virtual-full ground at Y=0, with empty and mixed allocated cells above it.
    # Allocated pages obey micro occupancy iff their fine byte is nonzero.
    grid = (3, 2, 4)
    cells, pages = {}, {}
    for i, coarse in enumerate(product(*(range(n) for n in grid))):
        if coarse[1] == 0:
            cells[coarse] = (7, INVALID_PAGE)
        elif (coarse[0] + coarse[2]) % 3 == 0:
            cells[coarse] = (0, INVALID_PAGE if i % 2 else i)
        else:
            cells[coarse] = (7, i)
        if cells[coarse][1] != INVALID_PAGE:
            pages[i] = make_page(rng.randrange(1, 256) if rng.getrandbits(1) else 0
                                 for _ in range(512))

    def load_byte(q):
        return load_fine_ao_byte(grid, cells, pages, q)

    def query(q):
        return solid_fine_global(grid, cells, pages, q)

    byte_checks = 0
    for micro in product(*(range(-1, n * 8 + 1) for n in grid)):
        expected = sum(int(query(add(tuple(x * 2 for x in micro), fine)))
                       << (fine[1] * 4 + fine[2] * 2 + fine[0])
                       for fine in product(range(2), repeat=3))
        check(load_byte(micro), expected, ("byte", micro))
        byte_checks += 1

    # Exhaust boundaries around fine/micro/coarse cells and the object exterior.
    positions = list(product(*[(-1, 0, 1, 14, 15, 16, 17, n * 16 - 1, n * 16)
                               for n in grid]))
    positions += [tuple(rng.randrange(-2, n * 16 + 2) for n in grid) for _ in range(1000)]
    hierarchy_checks = 0
    for p, (d1, d2) in product(positions, bases):
        check(fine_voxel_ao(p, d1, d2, load_byte), voxel_ao(p, d1, d2, query), (p, d1, d2))
        hierarchy_checks += 1
    known_bytes, known_ao = test_known_hit(grid, cells, pages, positions, bases)
    print(f"PASS: {stencils} exhaustive parity/basis/stencil cases with random surrounding bits, "
          f"{byte_checks} occupancy bytes, {hierarchy_checks} hierarchical AO cases.")
    print(f"PASS: {fine_edits} fine edits and {micro_edits} micro edits preserve occupancy iff "
          "fine byte != 0, including no-ops and neighboring packed data; negative control passed.")
    print(f"PASS: {known_bytes} known-hit byte comparisons and {known_ao} exposed-face AO cases, "
          "with exact coarse/page read routing and four gathers versus eight fine queries.")


if __name__ == "__main__":
    main()
