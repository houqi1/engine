// Include after the object/occupancy/direction-mask helpers, before main.
// Certifies an empty prefix for every pixel center in an 8x8 perspective tile.
// Requires current, conservative coarse/micro occupancy and affine object transforms.
// Roundoff pads assume ordinary finite FP32 arithmetic; retain a small world-space
// beamMargin for the main DDA's entry bias. The result is a bound, not a miss flag.
float conservativeBeamT(ivec2 beamPixel, ivec2 fullSize) {
    const float maxWorldT = 1.0e4;
    const float roundoff = 8.0e-6;
    const uint maxIntervals = 64u;
    const uint maxTileReads = 128u;
    const int maxIntervalTiles = 32;

    if (any(lessThanEqual(fullSize, ivec2(0))) || any(lessThan(beamPixel, ivec2(0)))) {
        return 0.0;
    }
    ivec2 first = beamPixel * 8;
    if (any(greaterThanEqual(first, fullSize))) {
        return 0.0;
    }
    ivec2 last = min(first + ivec2(7), fullSize - 1);
    vec2 projection = vec2(ubo.projX, ubo.projY);
    if (!all(greaterThan(abs(projection), vec2(0.0))) ||
        !all(lessThan(abs(projection), vec2(FLT_MAX)))) {
        return 0.0;
    }
    vec2 uv0 = (((vec2(first) + 0.5) / vec2(fullSize)) * 2.0 - 1.0) / projection;
    vec2 uv1 = (((vec2(last) + 0.5) / vec2(fullSize)) * 2.0 - 1.0) / projection;
    vec2 uvMin = min(uv0, uv1);
    vec2 uvMax = max(uv0, uv1);
    vec2 center = (uvMin + uvMax) * 0.5;
    vec2 radius = (uvMax - uvMin) * 0.5;
    vec2 uvAbs = max(abs(uvMin), abs(uvMax));

    // Never normalize corners: directions are affine in screen position at depth s.
    mat3 view = mat3(ubo.invView);
    vec3 worldCenter = view * vec3(center, -1.0);
    vec3 worldDx = view[0] * radius.x;
    vec3 worldDy = view[1] * radius.y;
    vec3 worldScale = abs(view[0]) * uvAbs.x + abs(view[1]) * uvAbs.y + abs(view[2]);
    vec3 worldRadius = abs(worldDx) + abs(worldDy) + roundoff * worldScale;
    // Distance to this direction AABB bounds every ray length from below. Unlike
    // min(corner lengths), this also covers interior minima and non-rigid invView.
    float minimumNorm = length(max(abs(worldCenter) - worldRadius, vec3(0.0))) * (1.0 - roundoff);
    if (!(minimumNorm > 1.0e-8 && minimumNorm < FLT_MAX)) {
        return 0.0;
    }
    float safeDepth = maxWorldT / minimumNorm;

    for (uint objectIndex = 0u; objectIndex < ubo.objectCount; ++objectIndex) {
        GpuVoxelObject o = objects[objectIndex];
        if ((o.flags & FLAG_ENABLED) == 0u || o.voxelSize <= 0.0 ||
            any(equal(o.gridSize, uvec3(0u))) || any(greaterThanEqual(o.occMin, o.occMax))) {
            continue;
        }
        // loadCoarseOcc4 uses cubic-grid strides. Unknown layouts cannot certify emptiness.
        if (!(o.voxelSize < FLT_MAX) || any(notEqual(o.gridSize, uvec3(o.gridSize.x)))) {
            return 0.0;
        }
        mat3 transform = mat3(o.worldToObject);
        mat3 absolute = mat3(abs(transform[0]), abs(transform[1]), abs(transform[2]));
        vec3 origin = (o.worldToObject * vec4(ubo.cameraPos, 1.0)).xyz / o.voxelSize;
        vec3 direction = (transform * worldCenter) / o.voxelSize;
        vec3 extent = (abs(transform * worldDx) + abs(transform * worldDy)) / o.voxelSize;
        vec3 directionPad = roundoff * (absolute * worldScale) / o.voxelSize;
        vec3 dMin = direction - extent - directionPad;
        vec3 dMax = direction + extent + directionPad;
        vec3 originPad = vec3(2.0e-4) + roundoff *
            (absolute * abs(ubo.cameraPos) + abs(o.worldToObject[3].xyz)) / o.voxelSize;
        vec3 boundsMin = o.occMin;
        vec3 boundsMax = o.occMax;
        if (!all(lessThan(abs(origin) + originPad + abs(dMin) + abs(dMax), vec3(FLT_MAX)))) {
            return 0.0;
        }

        // Necessary overlap inequalities for the entire beam at s >= 0:
        // origin + originPad + s*dMax >= occMin, origin - originPad + s*dMin <= occMax.
        float enter = 0.0;
        float exit = safeDepth;
        bool misses = false;
        for (int axis = 0; axis < 3; ++axis) {
            float lower = boundsMin[axis] - origin[axis] - originPad[axis];
            float upper = boundsMax[axis] - origin[axis] + originPad[axis];
            if (dMax[axis] > 0.0) {
                enter = max(enter, lower / dMax[axis]);
            } else if (dMax[axis] < 0.0) {
                exit = min(exit, lower / dMax[axis]);
            } else if (lower > 0.0) {
                misses = true;
            }
            if (dMin[axis] > 0.0) {
                exit = min(exit, upper / dMin[axis]);
            } else if (dMin[axis] < 0.0) {
                enter = max(enter, upper / dMin[axis]);
            } else if (upper < 0.0) {
                misses = true;
            }
        }
        if (!(abs(enter) < FLT_MAX && abs(exit) < FLT_MAX)) {
            return 0.0;
        }
        enter = max(0.0, enter - roundoff * (1.0 + abs(enter)));
        exit = min(safeDepth, exit + roundoff * (1.0 + abs(exit)));
        if (misses || enter > exit) {
            continue;
        }
        if (enter == exit || o.occMipWords == 0u) {
            safeDepth = min(safeDepth, enter);
            continue;
        }

        vec3 speed = max(abs(dMin), abs(dMax));
        float maxSpeed = max(speed.x, max(speed.y, speed.z));
        if (!(maxSpeed > 0.0)) {
            return 0.0;
        }
        float stepDepth = 4.0 / maxSpeed;
        float a = enter;
        uint readsLeft = maxTileReads;
        uint refinementsLeft = 2u;
        for (uint interval = 0u; interval < maxIntervals && a < exit; ++interval) {
            float b = min(exit, a + stepDepth);
            if (!(b > a)) {
                break;
            }
            vec3 pad = originPad + roundoff * (abs(origin) + b * speed + boundsMax + 1.0);
            vec3 boxMin = origin + min(a * dMin, b * dMin) - pad;
            vec3 boxMax = origin + max(a * dMax, b * dMax) + pad;
            if (!all(lessThan(abs(boxMin) + abs(boxMax), vec3(FLT_MAX)))) {
                return 0.0;
            }
            if (any(lessThan(boxMax, boundsMin)) || any(greaterThan(boxMin, boundsMax))) {
                a = b;
                continue;
            }
            // Inclusive floor after outward padding also keeps boundary-touching cells.
            ivec3 cellMin = ivec3(clamp(floor(boxMin), boundsMin, boundsMax - 1.0));
            ivec3 cellMax = ivec3(clamp(floor(boxMax), boundsMin, boundsMax - 1.0));
            ivec3 tileMin = cellMin >> 2;
            ivec3 tileMax = cellMax >> 2;
            ivec3 span = tileMax - tileMin + 1;
            if (any(greaterThan(span, ivec3(maxIntervalTiles)))) {
                break;
            }
            int tileCount = span.x * span.y * span.z;
            if (tileCount > maxIntervalTiles || uint(tileCount) > readsLeft) {
                break;
            }
            readsLeft -= uint(tileCount);
            bool blocked = false;
            ivec3 tile = tileMin;
            for (int j = 0; j < maxIntervalTiles && j < tileCount; ++j) {
                ivec3 tileOrigin = tile << 2;
                uvec2 occ = loadCoarseOcc4(o, tileOrigin);
                if ((occ.x | occ.y) != 0u) {
                    // Any occupied coarse in the swept 4^3 box blocks. dirReach AND plus
                    // nested brick tests skipped leftover occupancy after a cut, so the
                    // empty-prefix t0 jumped past the remaining hut/ground and DDA stored sky.
                    ivec3 lo = max(cellMin - tileOrigin, ivec3(0));
                    ivec3 hi = min(cellMax - tileOrigin, ivec3(3));
                    for (int z = lo.z; z <= hi.z && !blocked; ++z) {
                        for (int y = lo.y; y <= hi.y && !blocked; ++y) {
                            for (int x = lo.x; x <= hi.x && !blocked; ++x) {
                                uint bit = morton3Brick(ivec3(x, y, z));
                                uint word = bit < 32u ? occ.x : occ.y;
                                if ((word & (1u << (bit & 31u))) != 0u) {
                                    blocked = true;
                                }
                            }
                        }
                    }
                    if (blocked) {
                        break;
                    }
                }
                tile.x += 1;
                if (tile.x > tileMax.x) {
                    tile.x = tileMin.x;
                    tile.y += 1;
                    if (tile.y > tileMax.y) {
                        tile.y = tileMin.y;
                        tile.z += 1;
                    }
                }
            }
            if (blocked) {
                if (refinementsLeft == 0u) {
                    break;
                }
                stepDepth *= 0.5;
                refinementsLeft -= 1u;
            } else {
                a = b;
            }
        }
        // Only a completed sweep proves a miss. Every early stop retains its prefix.
        if (a < exit) {
            safeDepth = min(safeDepth, a);
        }
        if (safeDepth <= 0.0) {
            return 0.0;
        }
    }

    float worldT = min(maxWorldT, safeDepth * minimumNorm);
    return max(0.0, worldT - roundoff * (1.0 + worldT));
}
