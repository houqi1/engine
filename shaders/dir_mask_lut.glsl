// Inclusive 4^3 reach in Morton order: x0,y0,z0,x1,y1,z1.
// Octant bits 0,1,2 indicate negative X,Y,Z travel.
uvec2 directionReachMask(uint oct, ivec3 p) {
    // Lanes are X, Y, Z-low-word, Z-high-word. Reflect negative axes.
    uvec4 flip = uvec4(0u) - ((uvec4(oct) >> uvec4(0u, 1u, 2u, 2u)) & 1u);
    uvec4 q = (uvec4(p.x, p.y, p.z, p.z) ^ flip) & 3u;
    uvec4 low = (uvec4(0xAAAAAAAAu, 0xCCCCCCCCu, 0xF0F0F0F0u, 0xF0F0F0F0u) ^ flip)
                | ((q & 1u) - 1u);
    uvec4 high = uvec4(0xFF00FF00u, 0xFFFF0000u, 0x00000000u, 0xFFFFFFFFu) ^ flip;
    // Unsigned wraparound selects OR for positions 0/1, AND for 2/3.
    uvec4 selectOr = (q >> 1u) - 1u;
    uvec4 axis = (low & high) | ((low | high) & selectOr);
    return uvec2(axis.x & axis.y) & axis.zw;
}
