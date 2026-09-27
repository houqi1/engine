#pragma once

// Engine extension to Blast's stress solver. Work in the same mass/length scaled
// coordinates as B=I^(-1/2)C. A free component has six LEFT null modes; project
// these out of b, without adding anchors or changing the least-squares solution.
#include "bond.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

class StressEquilibrium
{
    using V = std::array<double, 3>;
    using W = std::array<double, 6>;
    struct Component
    {
        std::vector<uint32_t> nodes;
        bool anchored = false;
        double mass = 0;
        V center{};
        double inverse[3][3]{};
    };
    std::vector<V> positions, offsets;
    std::vector<double> sqrtMass, sqrtInertia;
    std::vector<Component> components;
    std::vector<W> work;
    bool dirty = true;
    static V cross(const V& a, const V& b)
    {
        return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
    }
    static V vec(const NvcVec3& p) { return {p.x,p.y,p.z}; }
    static W vec(const AngLin6& p) { return {p.ang.x,p.ang.y,p.ang.z,p.lin.x,p.lin.y,p.lin.z}; }

    void rebuild(const BondMatrixS& B)
    {
        if (!dirty) return;
        std::vector<uint32_t> parent(B.M);
        std::iota(parent.begin(), parent.end(), 0u);
        auto root = [&](uint32_t i) {
            while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
            return i;
        };
        for (uint32_t j=0; j<B.N; ++j) parent[root(B.C[j].node0)] = root(B.C[j].node1);
        std::vector<uint32_t> index(B.M, UINT32_MAX);
        components.clear();
        offsets.resize(B.M); sqrtMass.resize(B.M); sqrtInertia.resize(B.M); work.resize(B.M);
        for (uint32_t i=0; i<B.M; ++i)
        {
            const uint32_t r=root(i);
            if (index[r]==UINT32_MAX) { index[r]=static_cast<uint32_t>(components.size()); components.emplace_back(); }
            auto& c=components[index[r]];
            c.nodes.push_back(i);
            sqrtMass[i]=B.sqrt_I_inv[i].m>0 ? 1.0/B.sqrt_I_inv[i].m : 0;
            sqrtInertia[i]=B.sqrt_I_inv[i].I>0 ? 1.0/B.sqrt_I_inv[i].I : 0;
            c.anchored |= sqrtMass[i]==0 || sqrtInertia[i]==0;
            const double mass=sqrtMass[i]*sqrtMass[i];
            c.mass+=mass;
            for(int k=0;k<3;++k) c.center[k]+=mass*positions[i][k];
        }
        for (auto& c:components)
        {
            if(c.anchored || c.mass<=0) continue;
            for(double& x:c.center) x/=c.mass;
            double a[3][6]{};
            for(uint32_t i:c.nodes)
            {
                V& r=offsets[i];
                double r2=0;
                for(int k=0;k<3;++k) { r[k]=positions[i][k]-c.center[k]; r2+=r[k]*r[k]; }
                const double mass=sqrtMass[i]*sqrtMass[i], inertia=sqrtInertia[i]*sqrtInertia[i];
                for(int k=0;k<3;++k) for(int l=0;l<3;++l)
                    a[k][l]+=(k==l ? inertia+mass*r2 : 0)-mass*r[k]*r[l];
            }
            // Positive inertia at every free node makes this 3x3 tensor SPD.
            for(int k=0;k<3;++k) a[k][k+3]=1;
            for(int k=0;k<3;++k)
            {
                int pivot=k;
                for(int l=k+1;l<3;++l) if(std::abs(a[l][k])>std::abs(a[pivot][k])) pivot=l;
                for(int l=0;l<6;++l) std::swap(a[k][l],a[pivot][l]);
                const double d=a[k][k];
                for(int l=0;l<6;++l) a[k][l]/=d;
                for(int row=0;row<3;++row) if(row!=k)
                {
                    const double t=a[row][k];
                    for(int l=0;l<6;++l) a[row][l]-=t*a[k][l];
                }
            }
            for(int k=0;k<3;++k) for(int l=0;l<3;++l) c.inverse[k][l]=a[k][l+3];
        }
        dirty=false;
    }

    void projectWork(const BondMatrixS& B)
    {
        for(uint32_t i=0;i<B.M;++i) for(int k=0;k<3;++k)
        {
            if(sqrtInertia[i]==0) work[i][k]=0;
            if(sqrtMass[i]==0) work[i][k+3]=0;
        }
        for(const auto& c:components)
        {
            if(c.anchored || c.mass<=0) continue;
            V v{}, torque{}, omega{};
            for(uint32_t i:c.nodes)
            {
                V p{};
                for(int k=0;k<3;++k) { p[k]=sqrtMass[i]*work[i][k+3]; v[k]+=p[k]; }
                const V rxp=cross(offsets[i],p);
                // C uses M - offset x F. Consequently a rigid left-null mode
                // has linear=omega x r and angular=-omega (SDK convention).
                for(int k=0;k<3;++k) torque[k]+=rxp[k]-sqrtInertia[i]*work[i][k];
            }
            for(int k=0;k<3;++k)
            {
                v[k]/=c.mass;
                for(int l=0;l<3;++l) omega[k]+=c.inverse[k][l]*torque[l];
            }
            for(uint32_t i:c.nodes)
            {
                const V rotation=cross(omega,offsets[i]);
                for(int k=0;k<3;++k)
                {
                    work[i][k]+=sqrtInertia[i]*omega[k];
                    work[i][k+3]-=sqrtMass[i]*(v[k]+rotation[k]);
                }
            }
        }
    }
public:
    double relativeError = std::numeric_limits<double>::infinity();
    bool finite = true;
    void prepare(const SolverNodeS* nodes,uint32_t count,float lengthScale)
    {
        positions.resize(count);
        for(uint32_t i=0;i<count;++i) for(int k=0;k<3;++k)
            positions[i][k]=vec(nodes[i].CoM)[k]/lengthScale;
        dirty=true;
    }
    void topologyChanged() { dirty=true; }
    void project(AngLin6* b,const BondMatrixS& B)
    {
        rebuild(B);
        for(uint32_t i=0;i<B.M;++i) work[i]=vec(b[i]);
        projectWork(B);
        for(uint32_t i=0;i<B.M;++i)
        {
            b[i].ang={float(work[i][0]),float(work[i][1]),float(work[i][2])};
            b[i].lin={float(work[i][3]),float(work[i][4]),float(work[i][5])};
        }
    }
    bool check(const AngLin6* x,const AngLin6* b,const BondMatrixS& B,double relativeTolerance,double absoluteTolerance, AngLin6* refreshedResidual=nullptr)
    {
        rebuild(B);
        std::fill(work.begin(),work.end(),W{});
        // Independently accumulate B*x in double precision, rather than trust
        // the recursively updated single-precision CG residual.
        for(uint32_t j=0;j<B.N;++j)
        {
            const auto& c=B.C[j];
            const V f=vec(x[j].lin), m=vec(x[j].ang);
            const V r0=cross(vec(c.offset0),f),r1=cross(vec(c.offset1),f);
            for(int k=0;k<3;++k)
            {
                work[c.node0][k]+=m[k]-r0[k]; work[c.node1][k]-=m[k]-r1[k];
                work[c.node0][k+3]+=f[k]; work[c.node1][k+3]-=f[k];
            }
        }
        for(uint32_t i=0;i<B.M;++i)
        {
            const W rhs=vec(b[i]);
            for(int k=0;k<3;++k)
            {
                work[i][k]=rhs[k]-B.sqrt_I_inv[i].I*work[i][k];
                work[i][k+3]=rhs[k+3]-B.sqrt_I_inv[i].m*work[i][k+3];
            }
        }
        projectWork(B);
        if (refreshedResidual) for(uint32_t i=0;i<B.M;++i)
        {
            refreshedResidual[i].ang={float(work[i][0]),float(work[i][1]),float(work[i][2])};
            refreshedResidual[i].lin={float(work[i][3]),float(work[i][4]),float(work[i][5])};
        }
        bool passed=true;
        finite=true; relativeError=0;
        // Check each connected component independently: a heavily loaded actor
        // must not hide a small sibling's error in a family-wide norm.
        for(const auto& c:components)
        {
            double residual2=0,rhs2=0;
            for(uint32_t i:c.nodes)
            {
                const W rhs=vec(b[i]);
                for(int k=0;k<6;++k) { residual2+=work[i][k]*work[i][k]; rhs2+=rhs[k]*rhs[k]; }
            }
            if(!std::isfinite(residual2) || !std::isfinite(rhs2)) { finite=false; passed=false; relativeError=INFINITY; continue; }
            const double residual=std::sqrt(residual2), scale=std::sqrt(rhs2);
            relativeError=std::max(relativeError,residual/std::max(scale,absoluteTolerance));
            passed &= residual<=absoluteTolerance+relativeTolerance*scale;
        }
        return passed;
    }
};
