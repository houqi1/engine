// Include after vertexAo. microPos is in absolute microcell coordinates.
uint loadFineAoByte(GpuVoxelObject o, ivec3 microPos, ivec3 hitCoarse, uint hitPage) {
    ivec3 coarse = microPos >> 3;
    uint page = hitPage;
    // Most stencil samples remain in the already resolved hit brick.
    if (any(notEqual(coarse, hitCoarse))) {
        if (!insideGrid(o, coarse)) return 0u;
        CoarseCell cell = readCell(o, coarse);
        if (cell.material == 0u) return 0u;
        page = cell.brickPage;
    }
    if (page == INVALID_BRICK_PAGE) {
        return 255u;
    }
    uint bit = morton3Brick(microPos & ivec3(7));
    // Scene mutators maintain micro occupancy iff this fine byte is nonzero.
    uint packed = loadBrickWord(page, MICRO_WORDS + (bit >> 2));
    return (packed >> ((bit & 3u) * 8u)) & 255u;
}

uint fineAoBit(ivec3 p) {
    ivec3 fine = p & ivec3(1);
    return uint(fine.y * 4 + fine.z * 2 + fine.x);
}

// Fine space only; d1/d2 must be orthogonal signed unit axes (no DDA ties).
vec4 fineVoxelAo(GpuVoxelObject o, ivec3 aoPos, ivec3 d1, ivec3 d2, ivec3 hitCoarse, uint hitPage) {
    // The four corners occupy distinct microcells and cover the entire stencil.
    uvec4 bytes = uvec4(
        loadFineAoByte(o, (aoPos + d1 + d2) >> 1, hitCoarse, hitPage),
        loadFineAoByte(o, (aoPos - d1 + d2) >> 1, hitCoarse, hitPage),
        loadFineAoByte(o, (aoPos - d1 - d2) >> 1, hitCoarse, hitPage),
        loadFineAoByte(o, (aoPos + d1 - d2) >> 1, hitCoarse, hitPage));
    bool near1 = all(equal((aoPos + d1) >> 1, aoPos >> 1));
    bool near2 = all(equal((aoPos + d2) >> 1, aoPos >> 1));
    uvec4 sideBytes = uvec4(
        near2 ? bytes.x : bytes.w,
        near1 ? bytes.x : bytes.y,
        near2 ? bytes.y : bytes.z,
        near1 ? bytes.w : bytes.z);
    // +/- one have identical fine parity, so opposite sides share bit indices.
    uint bit1 = fineAoBit(aoPos + d1);
    uint bit2 = fineAoBit(aoPos + d2);
    vec4 side = vec4((sideBytes >> uvec4(bit1, bit2, bit1, bit2)) & 1u);
    vec4 corner = vec4((bytes >> fineAoBit(aoPos + d1 + d2)) & 1u);
    vec4 ao;
    ao.x = vertexAo(side.xy, corner.x);
    ao.y = vertexAo(side.yz, corner.y);
    ao.z = vertexAo(side.zw, corner.z);
    ao.w = vertexAo(side.wx, corner.w);
    return 1.0 - ao;
}
