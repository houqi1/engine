#version 450

layout(location = 0) flat in uint vGpuIndex;
layout(location = 0) out vec4 outObjectId;

void main() {
    // Stored as float(id+1); 0 = miss. Avoids R32_UINT color targets on MoltenVK.
    outObjectId = vec4(float(vGpuIndex + 1u), 0.0, 0.0, 1.0);
}
