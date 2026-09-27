#pragma once

// CGLS on B and B^T without forming the normal matrix. Keep the solution and
// Krylov vectors in double across budgeted calls; float bond output is only an
// export, never the authoritative iterate of a resumed solve.
#include "StressEquilibrium.h"
#include <climits>

class StressDoubleSolve
{
    using W=std::array<double,6>;
    std::vector<W> x,r,p,z,s;
    double gamma=0, threshold=0;
    bool valid=false;
    static W vec(const AngLin6& a) { return {a.ang.x,a.ang.y,a.ang.z,a.lin.x,a.lin.y,a.lin.z}; }
    static double norm2(const std::vector<W>& a)
    {
        double result=0;
        for(const W& v:a) for(double t:v) result+=t*t;
        return result;
    }
    static void multiply(std::vector<W>& out,const BondMatrixS& B,const std::vector<W>& in)
    {
        out.assign(B.M,W{});
        for(uint32_t j=0;j<B.N;++j)
        {
            const auto& c=B.C[j]; const W& f=in[j];
            for(int side=0;side<2;++side)
            {
                const auto& v=side ? c.offset1:c.offset0;
                W& n=out[side ? c.node1:c.node0]; const double sign=side ? -1:1;
                n[0]+=sign*(f[0]-(double(v.y)*f[5]-double(v.z)*f[4]));
                n[1]+=sign*(f[1]-(double(v.z)*f[3]-double(v.x)*f[5]));
                n[2]+=sign*(f[2]-(double(v.x)*f[4]-double(v.y)*f[3]));
                for(int k=3;k<6;++k) n[k]+=sign*f[k];
            }
        }
        for(uint32_t i=0;i<B.M;++i) for(int k=0;k<3;++k)
        { out[i][k]*=B.sqrt_I_inv[i].I; out[i][k+3]*=B.sqrt_I_inv[i].m; }
    }
    static void transpose(std::vector<W>& out,const BondMatrixS& B,const std::vector<W>& in)
    {
        out.assign(B.N,W{});
        for(uint32_t j=0;j<B.N;++j)
        {
            const auto& c=B.C[j]; W& n=out[j];
            for(int side=0;side<2;++side)
            {
                const uint32_t i=side ? c.node1:c.node0;
                const auto& v=side ? c.offset1:c.offset0;
                const double sign=side ? -1:1, a=B.sqrt_I_inv[i].I, l=B.sqrt_I_inv[i].m;
                const W& f=in[i];
                for(int k=0;k<3;++k) { n[k]+=sign*a*f[k]; n[k+3]+=sign*l*f[k+3]; }
                n[3]+=sign*a*(double(v.y)*f[2]-double(v.z)*f[1]);
                n[4]+=sign*a*(double(v.z)*f[0]-double(v.x)*f[2]);
                n[5]+=sign*a*(double(v.x)*f[1]-double(v.y)*f[0]);
            }
        }
    }
    void residual(const BondMatrixS& B,const AngLin6* b)
    {
        multiply(r,B,x);
        for(uint32_t i=0;i<B.M;++i) { const W rhs=vec(b[i]); for(int k=0;k<6;++k) r[i][k]=rhs[k]-r[i][k]; }
    }
public:
    void reset() { valid=false; }
    int solve(AngLin6* output,const BondMatrixS& B,const AngLin6* b,
              StressEquilibrium& equilibrium,AngLin6ErrorSq* error,double tolerance,
              double relativeTolerance,double absoluteTolerance,uint32_t maxIter,bool warm,bool hot)
    {
        if(!hot || !valid || x.size()!=B.N || r.size()!=B.M)
        {
            x.assign(B.N,W{});
            if(warm) for(uint32_t j=0;j<B.N;++j) x[j]=vec(output[j]);
            residual(B,b); transpose(z,B,r); p=z; gamma=norm2(z);
            double rhs2=0;
            for(uint32_t i=0;i<B.M;++i) for(double t:vec(b[i])) rhs2+=t*t;
            threshold=tolerance*tolerance*rhs2;
            valid=true;
        }
        auto accepted=[&]() { return equilibrium.check(x.data(),b,B,relativeTolerance,absoluteTolerance); };
        int result=-int(maxIter);
        for(uint32_t it=0;it<=maxIter;++it)
        {
            if(!std::isfinite(gamma)) { result=-INT_MAX; break; }
            if(gamma<=threshold)
            {
                if(accepted()) { result=int(it); break; }
                // Replace a drifted recursive residual without rounding x to float.
                if(gamma<=threshold*1.e-10)
                { residual(B,b); transpose(z,B,r); gamma=norm2(z); p=z; }
            }
            if(it==maxIter) break;
            multiply(s,B,p);
            const double denominator=norm2(s);
            if(!std::isfinite(denominator) || denominator<=0 || gamma<=0)
            { result=accepted() ? int(it):-INT_MAX; break; }
            const double alpha=gamma/denominator;
            if(!std::isfinite(alpha)) { result=-INT_MAX; break; }
            for(uint32_t j=0;j<B.N;++j) for(int k=0;k<6;++k) x[j][k]+=alpha*p[j][k];
            for(uint32_t i=0;i<B.M;++i) for(int k=0;k<6;++k) r[i][k]-=alpha*s[i][k];
            transpose(z,B,r);
            const double next=norm2(z), beta=next/gamma;
            for(uint32_t j=0;j<B.N;++j) for(int k=0;k<6;++k) p[j][k]=z[j][k]+beta*p[j][k];
            gamma=next;
        }
        // Always publish the independently accumulated residual, even on budget exit.
        accepted();
        if(!equilibrium.finite) result=-INT_MAX;
        if(error)
        {
            double a=0,l=0;
            for(const W& v:z) for(int k=0;k<3;++k) { a+=v[k]*v[k]; l+=v[k+3]*v[k+3]; }
            error->ang=float(a); error->lin=float(l);
        }
        for(uint32_t j=0;j<B.N;++j)
        { output[j].ang={float(x[j][0]),float(x[j][1]),float(x[j][2])}; output[j].lin={float(x[j][3]),float(x[j][4]),float(x[j][5])}; }
        valid=result!= -INT_MAX;
        return result;
    }
};
