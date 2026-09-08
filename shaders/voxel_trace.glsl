// Bounded multi-resolution DDA. Includer must declare:
//   readonly buffer CoarsePool { uvec2 cells[]; } coarsePool;
//   readonly buffer BrickPool { uint bricks[]; } brickSlabs[8];
//   readonly buffer OccupancyMip { uint occTiles[]; };
// and a uniform block named `ubo` with at least:
//   uint maxSteps, dirMaskCoarse, brickBitSkip, dirMaskBrick;
// Optional defines before include:
//   VE_TRACE_FORCE_NESTED / VE_TRACE_FORCE_FINE (1 = always on)
#include "voxel_types.glsl"
#include "dir_mask_lut.glsl"

bool insideGrid(GpuVoxelObject o, ivec3 p) {
    return all(greaterThanEqual(p, ivec3(0))) && all(lessThan(p, ivec3(o.gridSize)));
}

bool insideOcc(GpuVoxelObject o, ivec3 p) {
    return all(greaterThanEqual(p, ivec3(o.occMin))) && all(lessThan(p, ivec3(o.occMax)));
}

CoarseCell readCell(GpuVoxelObject o, ivec3 p) {
    uint n = o.gridSize.x;
    uint idx = o.voxelOffset + uint(p.x) + uint(p.y) * n + uint(p.z) * n * n;
    uvec2 packed = coarsePool.cells[idx];
    CoarseCell c;
    c.material = packed.x;
    c.brickPage = packed.y;
    return c;
}

uint loadBrickWord(uint page, uint wordIndex) {
    uint slab = page / PAGES_PER_SLAB;
    uint local = page % PAGES_PER_SLAB;
    uint idx = local * BRICK_PAGE_WORDS + wordIndex;
#ifdef VE_NONUNIFORM_BRICKS
    return brickSlabs[nonuniformEXT(slab)].bricks[idx];
#else
    if (slab == 0u) return brickSlabs[0].bricks[idx];
    if (slab == 1u) return brickSlabs[1].bricks[idx];
    if (slab == 2u) return brickSlabs[2].bricks[idx];
    if (slab == 3u) return brickSlabs[3].bricks[idx];
    if (slab == 4u) return brickSlabs[4].bricks[idx];
    if (slab == 5u) return brickSlabs[5].bricks[idx];
    if (slab == 6u) return brickSlabs[6].bricks[idx];
    return brickSlabs[7].bricks[idx];
#endif
}

uint morton3Brick(ivec3 p) {
    uint x = uint(p.x);
    uint y = uint(p.y);
    uint z = uint(p.z);
    uint m = 0u;
    m |= (x & 1u) << 0;
    m |= (y & 1u) << 1;
    m |= (z & 1u) << 2;
    m |= ((x >> 1) & 1u) << 3;
    m |= ((y >> 1) & 1u) << 4;
    m |= ((z >> 1) & 1u) << 5;
    m |= ((x >> 2) & 1u) << 6;
    m |= ((y >> 2) & 1u) << 7;
    m |= ((z >> 2) & 1u) << 8;
    return m;
}

bool getMicroVoxel(uint page, ivec3 c) {
    if (any(lessThan(c, ivec3(0))) || any(greaterThanEqual(c, ivec3(8)))) {
        return false;
    }
    if (page == INVALID_BRICK_PAGE) {
        return true;
    }
    uint bit = morton3Brick(c);
    uint word = loadBrickWord(page, bit >> 5);
    return ((word >> (bit & 31u)) & 1u) != 0u;
}

bool getFineVoxel(uint page, ivec3 micro, ivec3 fine) {
    if (any(lessThan(fine, ivec3(0))) || any(greaterThanEqual(fine, ivec3(2)))) {
        return false;
    }
    if (!getMicroVoxel(page, micro)) {
        return false;
    }
    if (page == INVALID_BRICK_PAGE) {
        return true;
    }
    uint microBit = morton3Brick(micro);
    uint packed = loadBrickWord(page, MICRO_WORDS + (microBit >> 2));
    uint byte = (packed >> ((microBit & 3u) * 8u)) & 255u;
    uint fbit = uint(fine.y * 4 + fine.z * 2 + fine.x);
    return ((byte >> fbit) & 1u) != 0u;
}

uint dirOctantFromSgn(vec3 sgn) {
    return (sgn.x < 0.0 ? 1u : 0u) | (sgn.y < 0.0 ? 2u : 0u) | (sgn.z < 0.0 ? 4u : 0u);
}

uvec2 dirReach4(uint oct, ivec3 p) {
    return directionReachMask(oct, p);
}

bool dirAndEmpty(uvec2 occ, uvec2 reach) {
    return (occ.x & reach.x) == 0u && (occ.y & reach.y) == 0u;
}

uvec2 loadCoarseOcc4(GpuVoxelObject o, ivec3 mapPos) {
    if (o.occMipWords == 0u) {
        return uvec2(0xFFFFFFFFu);
    }
    uint tileN = (o.gridSize.x + 3u) >> 2;
    uvec3 t = uvec3(mapPos) >> 2;
    uint tile = t.x + t.y * tileN + t.z * tileN * tileN;
    uint base = o.occMipOffset + tile * 2u;
    return uvec2(occTiles[base], occTiles[base + 1u]);
}

int coarseSkipShift(GpuVoxelObject o, ivec3 mapPos, uint oct) {
    if (ubo.dirMaskCoarse == 0u) {
        return -1;
    }
    uvec2 occ4 = loadCoarseOcc4(o, mapPos);
    if ((occ4.x | occ4.y) == 0u) {
        return 2;
    }
    if (dirAndEmpty(occ4, dirReach4(oct, mapPos))) {
        return 2;
    }
    return -1;
}

bvec3 stepMask(vec3 sideDist) {
    return lessThanEqual(sideDist.xyz, min(sideDist.yzx, sideDist.zxy));
}

bool jumpAlignedBox(inout ivec3 mapPos, inout vec3 sideDist, inout bvec3 mask, vec3 origin,
                    vec3 rd, vec3 invDir, vec3 sgn, vec3 deltaDist, ivec3 rayStep, int shift,
                    vec3 boundsMin, vec3 boundsMax, inout uint steps) {
    ivec3 macro = mapPos >> shift;
    vec3 boxMin = vec3(macro << shift);
    vec3 boxMax = min(boxMin + vec3(float(1 << shift)), boundsMax);
    vec3 t0m = (boxMin - origin) * invDir;
    vec3 t1m = (boxMax - origin) * invDir;
    vec3 tLargerM = max(t0m, t1m);
    float tExitM = min(min(tLargerM.x, tLargerM.y), tLargerM.z);
    bvec3 skipMask;
    if (tLargerM.x <= tLargerM.y && tLargerM.x <= tLargerM.z) {
        skipMask = bvec3(true, false, false);
    } else if (tLargerM.y <= tLargerM.z) {
        skipMask = bvec3(false, true, false);
    } else {
        skipMask = bvec3(false, false, true);
    }
    vec3 newPos = origin + rd * (tExitM + 1e-4);
    vec3 lo = boundsMin;
    vec3 hi = max(boundsMax - vec3(1.0), boundsMin);
    ivec3 newMap = ivec3(clamp(floor(newPos), lo, hi));
    steps += 1u;
    if (newMap == mapPos || (newMap >> shift) == macro) {
        mask = stepMask(sideDist);
        sideDist += vec3(mask) * deltaDist;
        mapPos += ivec3(vec3(mask)) * rayStep;
        return true;
    }
    if (any(lessThan(newMap, ivec3(boundsMin))) || any(greaterThanEqual(newMap, ivec3(boundsMax)))) {
        return false;
    }
    mapPos = newMap;
    mask = skipMask;
    sideDist = (sgn * (vec3(mapPos) - origin) + (sgn * 0.5 + 0.5)) * deltaDist;
    return true;
}

bool traceFineBounded(uint page, ivec3 micro, vec3 localPos, vec3 rd, vec3 sgn, bvec3 enterMask,
                      float tMaxLocal, out ivec3 outFine, out bvec3 outMask, out vec3 outUvw,
                      out float outT, inout uint outSteps) {
    localPos = clamp(localPos, vec3(0.0001), vec3(1.9999));
    ivec3 mapPos = ivec3(floor(localPos));
    vec3 deltaDist = abs(clamp(1.0 / rd, vec3(-FLT_MAX), vec3(FLT_MAX)));
    vec3 sideDist = (sgn * (vec3(mapPos) - localPos) + (sgn * 0.5 + 0.5)) * deltaDist;
    sideDist = mix(sideDist, vec3(FLT_MAX), equal(rd, vec3(0.0)));
    bvec3 mask = enterMask;
    ivec3 rayStep = ivec3(sgn);
    outUvw = clamp(localPos - vec3(mapPos), vec3(0.0), vec3(1.0));
    outT = 0.0;

    uint microBit = morton3Brick(micro);
    uint packed = loadBrickWord(page, MICRO_WORDS + (microBit >> 2));
    uint fineBits = (packed >> ((microBit & 3u) * 8u)) & 255u;

    if ((fineBits & (1u << uint(mapPos.y * 4 + mapPos.z * 2 + mapPos.x))) != 0u) {
        outFine = mapPos;
        outMask = mask;
        return true;
    }

    for (int i = 0; i < 8; ++i) {
        outSteps += 1u;
        float tHit = min(sideDist.x, min(sideDist.y, sideDist.z));
        if (tHit > tMaxLocal) {
            return false;
        }
        mask = stepMask(sideDist);
        sideDist += vec3(mask) * deltaDist;
        mapPos += ivec3(vec3(mask)) * rayStep;
        if (any(lessThan(mapPos, ivec3(0))) || any(greaterThanEqual(mapPos, ivec3(2)))) {
            return false;
        }
        if ((fineBits & (1u << uint(mapPos.y * 4 + mapPos.z * 2 + mapPos.x))) != 0u) {
            vec3 hitPos = localPos + rd * tHit;
            outUvw = clamp(hitPos - vec3(mapPos), vec3(0.0), vec3(1.0));
            outFine = mapPos;
            outMask = mask;
            outT = tHit;
            return true;
        }
    }
    return false;
}

bool traceMicroBounded(uint page, vec3 localPos, vec3 rd, vec3 sgn, bvec3 enterMask, bool useFine,
                       float startT, float tMaxMicro, out ivec3 outMicro, out ivec3 outFine,
                       out bvec3 outMask, out vec3 outUvw, out float outTMicro, out float outTFine,
                       inout uint outSteps) {
    localPos = clamp(localPos, vec3(0.0001), vec3(7.9999));
    ivec3 mapPos = ivec3(floor(localPos + rd * startT));
    vec3 invDir = clamp(1.0 / rd, vec3(-FLT_MAX), vec3(FLT_MAX));
    vec3 deltaDist = abs(invDir);
    vec3 sideDist = (sgn * (vec3(mapPos) - localPos) + (sgn * 0.5 + 0.5)) * deltaDist;
    bvec3 mask = enterMask;
    ivec3 rayStep = ivec3(sgn);
    outTMicro = 0.0;
    outTFine = 0.0;
    outFine = ivec3(0);

    vec3 t0b = (vec3(0.0) - localPos) * invDir;
    vec3 t1b = (vec3(8.0) - localPos) * invDir;
    float tExitBrick = min(min(max(t0b.x, t1b.x), max(t0b.y, t1b.y)), max(t0b.z, t1b.z));
    tExitBrick = min(tExitBrick, tMaxMicro);
    uint oct = dirOctantFromSgn(sgn);

    uint cachedOctant = 8u;
    uvec2 occ4 = uvec2(0u);

    for (int i = 0; i < 48; ++i) {
        if (any(lessThan(mapPos, ivec3(0))) || any(greaterThanEqual(mapPos, ivec3(8)))) {
            return false;
        }

        uint microBit = morton3Brick(mapPos);
        uint occupancy;
        if (ubo.brickBitSkip != 0u) {
            uint octant = microBit >> 6;
            if (cachedOctant != octant) {
                occ4 = uvec2(loadBrickWord(page, octant * 2u),
                             loadBrickWord(page, octant * 2u + 1u));
                cachedOctant = octant;
            }
            occupancy = (microBit & 32u) != 0u ? occ4.y : occ4.x;
        } else {
            occupancy = loadBrickWord(page, microBit >> 5);
        }
        bool occupied = ((occupancy >> (microBit & 31u)) & 1u) != 0u;
        if (occupied) {
            vec3 mini = ((vec3(mapPos) - localPos) + 0.5 - 0.5 * sgn) * invDir;
            float tHit = max(mini.x, max(mini.y, mini.z));
            if (tHit > tExitBrick) {
                return false;
            }
            vec3 local01;
            if (tHit <= 0.0) {
                tHit = 0.0;
                local01 = clamp(localPos - vec3(mapPos), vec3(0.0), vec3(0.9999));
            } else {
                vec3 hitPos = localPos + rd * tHit;
                local01 = clamp(hitPos - vec3(mapPos), vec3(0.0), vec3(0.9999));
            }
            if (!useFine) {
                outMicro = mapPos;
                outMask = mask;
                outUvw = local01;
                outTMicro = tHit;
                return true;
            }
            float fineBudget = (tExitBrick - tHit) * 2.0;
            if (traceFineBounded(page, mapPos, local01 * 2.0, rd, sgn, mask, fineBudget, outFine,
                                 outMask, outUvw, outTFine, outSteps)) {
                outMicro = mapPos;
                outTMicro = tHit;
                return true;
            }
        }

        int skipShift = -1;
        if (ubo.brickBitSkip != 0u && !occupied) {
            if ((occ4.x | occ4.y) == 0u ||
                (ubo.dirMaskBrick != 0u && dirAndEmpty(occ4, dirReach4(oct, mapPos)))) {
                skipShift = 2;
            } else if (((occupancy >> (microBit & 24u)) & 255u) == 0u) {
                skipShift = 1;
            }
        }
        if (skipShift >= 0) {
            ivec3 macro = mapPos >> skipShift;
            vec3 boxMin = vec3(macro << skipShift);
            vec3 boxMax = min(boxMin + vec3(float(1 << skipShift)), vec3(8.0));
            vec3 t0m = (boxMin - localPos) * invDir;
            vec3 t1m = (boxMax - localPos) * invDir;
            vec3 tLargerM = max(t0m, t1m);
            float tExitM = min(min(tLargerM.x, tLargerM.y), tLargerM.z);
            if (tExitM >= tExitBrick - 1e-5) {
                return false;
            }
            bvec3 skipMask;
            if (tLargerM.x <= tLargerM.y && tLargerM.x <= tLargerM.z) {
                skipMask = bvec3(true, false, false);
            } else if (tLargerM.y <= tLargerM.z) {
                skipMask = bvec3(false, true, false);
            } else {
                skipMask = bvec3(false, false, true);
            }
            vec3 newPos = localPos + rd * (tExitM + 1e-4);
            ivec3 newMap = ivec3(clamp(floor(newPos), vec3(0.0), vec3(7.0)));
            outSteps += 1u;
            if (newMap == mapPos || (newMap >> skipShift) == macro) {
                mask = stepMask(sideDist);
                sideDist += vec3(mask) * deltaDist;
                mapPos += ivec3(vec3(mask)) * rayStep;
                continue;
            }
            mapPos = newMap;
            mask = skipMask;
            sideDist = (sgn * (vec3(mapPos) - localPos) + (sgn * 0.5 + 0.5)) * deltaDist;
            continue;
        }

        outSteps += 1u;
        float tNext = min(sideDist.x, min(sideDist.y, sideDist.z));
        if (tNext > tExitBrick) {
            return false;
        }
        mask = stepMask(sideDist);
        sideDist += vec3(mask) * deltaDist;
        mapPos += ivec3(vec3(mask)) * rayStep;
    }
    return false;
}

struct TraceHit {
    bool hit;
    float tWorld;
    ivec3 mapPos;
    ivec3 micro;
    ivec3 fine;
    bvec3 mask;
    vec3 sgn;
    vec3 uvw;
    uint material;
    uint page;
    uint steps;
    bool usedMicro;
    bool usedFine;
};

// World-space ray vs object occupancy AABB. Returns false if no overlap.
bool rayOccBoundsWorld(GpuVoxelObject o, vec3 Ow, vec3 Dw, out float tEnterW, out float tExitW) {
    tEnterW = 0.0;
    tExitW = 0.0;
    if ((o.flags & FLAG_ENABLED) == 0u || o.voxelSize <= 0.0) {
        return false;
    }
    vec3 Ol = (o.worldToObject * vec4(Ow, 1.0)).xyz;
    vec3 Dl = mat3(o.worldToObject) * Dw;
    if (dot(Dl, Dl) < 1e-12) {
        return false;
    }
    vec3 ro = Ol / o.voxelSize;
    vec3 rd = Dl;
    vec3 invDir = clamp(1.0 / rd, vec3(-FLT_MAX), vec3(FLT_MAX));
    vec3 boundsMin = o.occMin;
    vec3 boundsMax = o.occMax;
    if (any(greaterThanEqual(boundsMin, boundsMax))) {
        return false;
    }
    vec3 t0 = (boundsMin - ro) * invDir;
    vec3 t1 = (boundsMax - ro) * invDir;
    float tEnter = max(max(min(t0.x, t1.x), min(t0.y, t1.y)), max(min(t0.z, t1.z), 0.0));
    float tExit = min(min(max(t0.x, t1.x), max(t0.y, t1.y)), max(t0.z, t1.z));
    if (tEnter > tExit) {
        return false;
    }
    tEnterW = tEnter * o.voxelSize;
    tExitW = tExit * o.voxelSize;
    return true;
}

TraceHit traceObjectBounded(GpuVoxelObject o, vec3 Ow, vec3 Dw, float tWorldMin, float tWorldMax) {
    TraceHit result;
    result.hit = false;
    result.tWorld = FLT_MAX;
    result.mapPos = ivec3(0);
    result.micro = ivec3(0);
    result.fine = ivec3(0);
    result.mask = bvec3(false);
    result.sgn = vec3(1.0);
    result.uvw = vec3(0.5);
    result.material = 0u;
    result.page = INVALID_BRICK_PAGE;
    result.steps = 0u;
    result.usedMicro = false;
    result.usedFine = false;

    if ((o.flags & FLAG_ENABLED) == 0u || o.voxelSize <= 0.0) {
        return result;
    }

    vec3 Ol = (o.worldToObject * vec4(Ow, 1.0)).xyz;
    // Keep |Dl| so local t scales to world meters (do not renormalize).
    vec3 Dl = mat3(o.worldToObject) * Dw;
    if (dot(Dl, Dl) < 1e-12) {
        return result;
    }

    vec3 ro = Ol / o.voxelSize;
    vec3 rd = Dl;

    vec3 invDir = clamp(1.0 / rd, vec3(-FLT_MAX), vec3(FLT_MAX));
    vec3 sgn = sign(rd);
    if (sgn.x == 0.0) sgn.x = 1.0;
    if (sgn.y == 0.0) sgn.y = 1.0;
    if (sgn.z == 0.0) sgn.z = 1.0;
    result.sgn = sgn;
    uint oct = dirOctantFromSgn(sgn);

    vec3 boundsMin = o.occMin;
    vec3 boundsMax = o.occMax;
    if (any(greaterThanEqual(boundsMin, boundsMax))) {
        return result;
    }
    vec3 t0 = (boundsMin - ro) * invDir;
    vec3 t1 = (boundsMax - ro) * invDir;
    vec3 tSmaller = min(t0, t1);
    vec3 tLarger = max(t0, t1);
    float tEnter = max(max(tSmaller.x, tSmaller.y), max(tSmaller.z, 0.0));
    float tExit = min(min(tLarger.x, tLarger.y), tLarger.z);
    float gridEnter = tEnter;
    tEnter = max(tEnter, tWorldMin / max(o.voxelSize, 1e-6));
    tExit = min(tExit, tWorldMax / max(o.voxelSize, 1e-6));
    if (tEnter > tExit) {
        return result;
    }

    vec3 pos = ro + rd * (tEnter + 1e-4);
    ivec3 mapPos = ivec3(clamp(floor(pos), boundsMin, max(boundsMax - vec3(1.0), boundsMin)));
    vec3 gridEntryPos = ro + rd * (gridEnter + 1e-4);
    ivec3 startPos = ivec3(clamp(floor(gridEntryPos), boundsMin, max(boundsMax - vec3(1.0), boundsMin)));

    vec3 deltaDist = abs(invDir);
    vec3 sideDist = (sgn * (vec3(mapPos) - ro) + (sgn * 0.5 + 0.5)) * deltaDist;
    ivec3 rayStep = ivec3(sgn);

    bvec3 mask = bvec3(false);
    if (tSmaller.x > tSmaller.y && tSmaller.x > tSmaller.z) {
        mask = bvec3(true, false, false);
    } else if (tSmaller.y > tSmaller.z) {
        mask = bvec3(false, true, false);
    } else {
        mask = bvec3(false, false, true);
    }

    bool hit = false;
    uint matId = 0u;
    uint hitPage = INVALID_BRICK_PAGE;
    ivec3 microPos = ivec3(0);
    ivec3 finePos = ivec3(0);
    bool usedMicro = false;
    bool usedFine = false;
    vec3 hitUvw = vec3(0.5);
    float tHitLocal = tEnter;
    uint steps = 0u;

#if defined(VE_TRACE_FORCE_NESTED)
    bool nested = true;
#else
    bool nested = (o.flags & FLAG_NESTED) != 0u;
#endif
#if defined(VE_TRACE_FORCE_FINE)
    bool nestedFine = nested;
#else
    bool nestedFine = nested && (o.flags & FLAG_NESTED_FINE) != 0u;
#endif
    bool allowBrick = nested;
    bool allowFine = allowBrick && nestedFine;

    for (uint i = 0u; i < ubo.maxSteps; ++i) {
        steps += 1u;
        if (!insideOcc(o, mapPos) || !insideGrid(o, mapPos)) {
            break;
        }

        // Bound every coarse visit to the ray-box exit (world-comparable local t).
        {
            vec3 mini = ((vec3(mapPos) - ro) + 0.5 - 0.5 * sgn) * invDir;
            float tCellEnter = max(mini.x, max(mini.y, mini.z));
            if (mapPos == startPos) {
                tCellEnter = gridEnter;
            }
            if (tCellEnter > tExit) {
                break;
            }
        }

        CoarseCell cell = readCell(o, mapPos);
        matId = cell.material;
        if (matId != 0u) {
            hitPage = cell.brickPage;
            vec3 local01;
            float tCoarse = gridEnter;
            if (mapPos == startPos) {
                local01 = clamp(gridEntryPos - vec3(mapPos), vec3(0.0), vec3(0.9999));
            } else {
                vec3 mini = ((vec3(mapPos) - ro) + 0.5 - 0.5 * sgn) * invDir;
                tCoarse = max(mini.x, max(mini.y, mini.z));
                vec3 intersect = ro + rd * tCoarse;
                local01 = clamp(intersect - vec3(mapPos), vec3(0.0), vec3(0.9999));
            }

            bool hasBrick = cell.brickPage != INVALID_BRICK_PAGE;
            if (!allowBrick || !hasBrick) {
                tHitLocal = tCoarse;
                hit = true;
                if (allowBrick && !hasBrick) {
                    vec3 microF = clamp(local01 * 8.0, vec3(0.0), vec3(7.9999));
                    microPos = ivec3(floor(microF));
                    usedMicro = true;
                    if (allowFine) {
                        vec3 fineF = clamp((microF - vec3(microPos)) * 2.0, vec3(0.0), vec3(1.9999));
                        finePos = ivec3(floor(fineF));
                        hitUvw = clamp(fineF - vec3(finePos), vec3(0.0), vec3(1.0));
                        usedFine = true;
                    } else {
                        hitUvw = clamp(microF - vec3(microPos), vec3(0.0), vec3(1.0));
                    }
                } else {
                    hitUvw = local01;
                }
                break;
            }

            ivec3 hitMicro;
            ivec3 hitFine;
            bvec3 microMask;
            vec3 microUvw;
            float tMicro = 0.0;
            float tFine = 0.0;
            float microBudget = (tExit - tCoarse) * 8.0;
            bool brickHit = traceMicroBounded(cell.brickPage, local01 * 8.0, rd, sgn, mask, allowFine,
                                              max(0.0, (tEnter - tCoarse) * 8.0), microBudget,
                                              hitMicro, hitFine, microMask, microUvw, tMicro, tFine,
                                              steps);
            if (brickHit) {
                hit = true;
                usedMicro = true;
                usedFine = allowFine;
                microPos = hitMicro;
                finePos = hitFine;
                mask = microMask;
                hitUvw = microUvw;
                tHitLocal = tCoarse + tMicro / 8.0 + tFine / 16.0;
                break;
            }
        }

        if (matId == 0u) {
            int skipShift = coarseSkipShift(o, mapPos, oct);
            if (skipShift >= 0) {
                ivec3 macro = mapPos >> skipShift;
                vec3 boxMin = vec3(macro << skipShift);
                vec3 boxMax = min(boxMin + vec3(float(1 << skipShift)), boundsMax);
                vec3 t0m = (boxMin - ro) * invDir;
                vec3 t1m = (boxMax - ro) * invDir;
                float tExitM = min(min(max(t0m.x, t1m.x), max(t0m.y, t1m.y)), max(t0m.z, t1m.z));
                if (tExitM > tExit) {
                    break;
                }
                if (!jumpAlignedBox(mapPos, sideDist, mask, ro, rd, invDir, sgn, deltaDist, rayStep,
                                    skipShift, boundsMin, boundsMax, steps)) {
                    break;
                }
                continue;
            }
        }

        float tNext = min(sideDist.x, min(sideDist.y, sideDist.z));
        if (tNext > tExit) {
            break;
        }
        mask = stepMask(sideDist);
        sideDist += vec3(mask) * deltaDist;
        mapPos += ivec3(vec3(mask)) * rayStep;
    }

    result.steps = steps;
    result.mask = mask;
    if (hit) {
        vec3 hitLocalMeters = (ro + rd * tHitLocal) * o.voxelSize;
        vec3 hitWorld = (o.objectToWorld * vec4(hitLocalMeters, 1.0)).xyz;
        result.tWorld = dot(hitWorld - Ow, Dw);
        result.hit = true;
        result.mapPos = mapPos;
        result.micro = microPos;
        result.fine = finePos;
        result.uvw = hitUvw;
        result.material = matId;
        result.page = hitPage;
        result.usedMicro = usedMicro;
        result.usedFine = usedFine;
    }
    return result;
}

uint readFinePackedRgb(uint page, ivec3 micro, ivec3 fine) {
    uint microBit = morton3Brick(micro);
    uint fbit = uint(fine.y * 4 + fine.z * 2 + fine.x);
    uint fi = microBit * 8u + fbit;
    return loadBrickWord(page, FINE_COLOR_OFFSET + fi);
}

vec3 unpackRgb888(uint p) {
    return vec3(float((p >> 16u) & 255u), float((p >> 8u) & 255u), float(p & 255u)) / 255.0;
}

vec3 albedoForVoxel(GpuVoxelObject o, uint matId, uint page, ivec3 micro, ivec3 fine,
                    bool usedMicro, bool usedFine) {
    if ((o.flags & FLAG_IMPORT_PALETTE) != 0u && page != INVALID_BRICK_PAGE) {
        if (usedFine) {
            uint packed = readFinePackedRgb(page, micro, fine);
            if ((packed >> 24u) != 0u) {
                return unpackRgb888(packed);
            }
        } else if (usedMicro) {
            for (int i = 0; i < 8; ++i) {
                ivec3 f = ivec3(i & 1, (i >> 2) & 1, (i >> 1) & 1);
                if (getFineVoxel(page, micro, f)) {
                    uint packed = readFinePackedRgb(page, micro, f);
                    if ((packed >> 24u) != 0u) {
                        return unpackRgb888(packed);
                    }
                }
            }
        }
    }
    return vec3(0.62, 0.64, 0.68);
}

bool solidCoarse(GpuVoxelObject o, ivec3 p) {
    return insideGrid(o, p) && readCell(o, p).material != 0u;
}

bool solidMicroGlobal(GpuVoxelObject o, ivec3 g) {
    ivec3 coarse = g >> 3;
    ivec3 local = g & ivec3(7);
    if (!insideGrid(o, coarse)) {
        return false;
    }
    CoarseCell cell = readCell(o, coarse);
    if (cell.material == 0u) {
        return false;
    }
    if (cell.brickPage == INVALID_BRICK_PAGE) {
        return true;
    }
    return getMicroVoxel(cell.brickPage, local);
}

bool solidFineGlobal(GpuVoxelObject o, ivec3 g) {
    ivec3 coarse = g >> 4;
    ivec3 micro = (g >> 1) & ivec3(7);
    ivec3 fine = g & ivec3(1);
    if (!insideGrid(o, coarse)) {
        return false;
    }
    CoarseCell cell = readCell(o, coarse);
    if (cell.material == 0u) {
        return false;
    }
    return getFineVoxel(cell.brickPage, micro, fine);
}

bool neighborSolid(GpuVoxelObject o, ivec3 solidPos, ivec3 offset, uint space) {
    ivec3 p = solidPos + offset;
    if (space == 2u) {
        return solidFineGlobal(o, p);
    }
    return space == 1u ? solidMicroGlobal(o, p) : solidCoarse(o, p);
}

float vertexAo(vec2 side, float corner) {
    return (side.x + side.y + max(corner, side.x * side.y)) / 3.0;
}

#include "voxel_ao.glsl"

vec4 voxelAo(GpuVoxelObject o, ivec3 aoPos, ivec3 d1, ivec3 d2, uint space) {
    vec4 side = vec4(
        float(neighborSolid(o, aoPos, d1, space)),
        float(neighborSolid(o, aoPos, d2, space)),
        float(neighborSolid(o, aoPos, -d1, space)),
        float(neighborSolid(o, aoPos, -d2, space)));
    vec4 corner = vec4(
        float(neighborSolid(o, aoPos, d1 + d2, space)),
        float(neighborSolid(o, aoPos, -d1 + d2, space)),
        float(neighborSolid(o, aoPos, -d1 - d2, space)),
        float(neighborSolid(o, aoPos, d1 - d2, space)));
    vec4 ao;
    ao.x = vertexAo(side.xy, corner.x);
    ao.y = vertexAo(side.yz, corner.y);
    ao.z = vertexAo(side.zw, corner.z);
    ao.w = vertexAo(side.wx, corner.w);
    return 1.0 - ao;
}

vec2 faceUv(bvec3 mask, vec3 uvw) {
    return vec2(dot(vec3(mask) * uvw.yzx, vec3(1.0)),
                dot(vec3(mask) * uvw.zxy, vec3(1.0)));
}

float evalVoxelAo(GpuVoxelObject o, ivec3 solidPos, bvec3 mask, vec3 sgn, vec3 uvw, uint space,
                  uint hitPage) {
    ivec3 aoPos = solidPos + ivec3(-vec3(mask) * sgn);
    ivec3 d1 = ivec3(vec3(mask).zxy);
    ivec3 d2 = ivec3(vec3(mask).yzx);
    vec4 corners = (space == 2u && int(mask.x) + int(mask.y) + int(mask.z) == 1)
        ? fineVoxelAo(o, aoPos, d1, d2, solidPos >> 4, hitPage)
        : voxelAo(o, aoPos, d1, d2, space);
    vec2 uv = clamp(faceUv(mask, uvw), vec2(0.0), vec2(1.0));
    float ao = mix(mix(corners.z, corners.w, uv.x), mix(corners.y, corners.x, uv.x), uv.y);
    ao = pow(clamp(ao, 0.0, 1.0), max(ubo.aoPower, 1e-3));
    return mix(1.0, ao, clamp(ubo.aoStrength, 0.0, 1.0));
}
