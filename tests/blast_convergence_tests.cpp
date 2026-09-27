#include "blast/BlastMemory.h"
#include "blast/ObjectOccupancyView.h"
#include "blast/StructureWorld.h"
#include "stress.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
constexpr float dt=1.0f/60.0f;
constexpr VoxelObjectId object{1,1}, ground{0,1};

void eccentricSupport(blast::BlastRuntime& runtime, float normalTolerance) {
  blast::StructureWorld world;
  require(world.init(runtime), "init");
  world.setSolverAccuracy(normalTolerance,1e-4f);
  blast::ObjectOccupancyView occupancy(64,0.1f,600.0f,[](int x,int y,int z) {
    return x<32 && z<32 && (y<4 || (y>=12 && y<16) || (x>=2 && x<4 && z>=2 && z<4 && y<12));
  },nullptr);
  blast::OccupancySampleOpts opts;
  opts.agg=2; opts.allowFloating=true;
  blast::StructureMountDesc desc;
  desc.objectId=object; desc.material.strengthPa=4e6f;
  desc.material.ownStrengthPa=4e6f; desc.material.fractureEnabled=true; desc.material.solverIters=200;
  require(world.mount(desc,blast::sampleOccupancy(occupancy,opts))==blast::BlastError::Ok,"mount");
  world.setContactLoadsEnabled(true); world.setImpactDamageEnabled(false);
  auto* inst=world.find(object);
  const auto& body=inst->bindings.front();
  std::vector<blast::WorldContactImpulse> contacts;
  for(float x:{0.1f,3.1f}) for(float z:{0.1f,3.1f}) {
    blast::WorldContactImpulse p;
    p.idA=object; p.idB=ground; p.persistent=true; p.xA=body.comAsset;
    p.worldPoint={x,0,z}; p.JA={0,body.mass*9.81f*dt/4,0}; contacts.push_back(p);
  }
  blast::BodyKinematics kin[2];
  kin[0].objectId=object; kin[1].objectId=ground; kin[1].awake=false;
  int ticks=0;
  for(int i=1;i<=600;++i) {
    if(i==1) world.setSolverIters(1,inst);
    if(i==2) world.setSolverIters(200,inst);
    world.onPhysicsTick(i,dt,contacts.data(),uint32_t(contacts.size()),kin,2);
    if(i==1) require(inst->blast.solver->getSolveStatus()==Nv::Blast::ExtStressSolver::SolveStatus::IterationLimit,
                    "exhausted iteration budget reported as convergence");
    ticks=i;
    if(i>=120 && inst->debug.converged) break;
  }
  std::printf("column solve tol=%g ticks=%d status=%d residual=%g lin=%g ang=%g maxC=%g\n",normalTolerance,ticks,int(inst->blast.solver->getSolveStatus()),inst->blast.solver->getEquilibriumError(),inst->debug.linErr,inst->debug.angErr,inst->debug.maxCompression);
  require(inst->debug.converged,"eccentric column did not converge");
  require(inst->blast.solver->getSolveStatus()==Nv::Blast::ExtStressSolver::SolveStatus::Converged,"convergence status");
  std::vector<Nv::Blast::ExtStressSolver::BondImpulse> impulses(inst->graph.bonds.size());
  impulses.resize(inst->blast.solver->copyBondImpulses(impulses.data(),uint32_t(impulses.size())));
  const auto* bonds=NvBlastAssetGetBonds(inst->blast.asset,nullptr);
  int cutCount=0;
  for(const auto& p:impulses) {
    const auto& bond=bonds[p.blastBondIndex];
    if(std::abs(bond.centroid[1]-0.8f)>1e-5f) continue;
    ++cutCount;
    const float expectedForce=(2457.6f+9.6f)*9.81f;
    const float expectedMoment=2457.6f*9.81f*1.3f;
    std::printf("tol=%g ticks=%d F=%.2f expected=%.2f Mx=%.2f Mz=%.2f expected=%.2f residual=%g candidates=%u\n",
      normalTolerance,ticks,std::abs(p.linear.y),expectedForce,std::abs(p.angular.x),std::abs(p.angular.z),expectedMoment,
      inst->blast.solver->getEquilibriumError(),inst->debug.candidateCount);
    require(std::abs(std::abs(p.linear.y)/expectedForce-1)<0.01f,"cut force error exceeds 1%");
    require(std::abs(std::abs(p.angular.x)/expectedMoment-1)<0.01f,"cut Mx error exceeds 1%");
    require(std::abs(std::abs(p.angular.z)/expectedMoment-1)<0.01f,"cut Mz error exceeds 1%");
  }
  require(cutCount==1,"expected one mid-column bond");
  require(inst->debug.candidateCount>0,"overloaded support must produce fracture candidates");
  // Sleep keeps the load; convergence must remain valid with the same RHS.
  kin[0].awake=false;
  world.onPhysicsTick(601,dt,nullptr,0,kin,2);
  require(inst->debug.converged,"sleep lost equilibrium");
  // A changed RHS invalidates the retained Krylov state. Halving every support
  // reaction must halve the solved internal loads after the smoothing settles.
  kin[0].awake=true;
  for(auto& p:contacts) p.JA*=0.5f;
  for(int i=602;i<=1201;++i) {
    world.onPhysicsTick(i,dt,contacts.data(),uint32_t(contacts.size()),kin,2);
    if(i>=722 && inst->debug.converged) break;
  }
  require(inst->debug.converged,"changed load did not converge");
  inst->blast.solver->copyBondImpulses(impulses.data(),uint32_t(impulses.size()));
  for(const auto& p:impulses) if(std::abs(bonds[p.blastBondIndex].centroid[1]-0.8f)<1e-5f)
    require(std::abs(std::abs(p.linear.y)/(0.5f*(2457.6f+9.6f)*9.81f)-1)<0.01f,"stale Krylov state after load change");
  world.shutdown();
}

void rigidModesAndSplit() {
  // Unequal masses, two free islands and one anchored island. Rotation uses
  // the SDK's angular sign convention, tested against its actual B operator.
  SolverNodeS nodes[6]={{{0,0,0},1,0.1f},{{1,0,0},3,0.2f},{{0,3,0},2,0.15f},{{1,3,0},4,0.3f},{{0,6,0},0,0},{{1,6,0},2,0.2f}};
  SolverBond bonds[3]={{{0.5f,0,0},{0,1}},{{0.5f,3,0},{2,3}},{{0.5f,6,0},{4,5}}};
  StressProcessor processor;
  StressProcessor::DataParams data;
  processor.prepare(nodes,6,bonds,3,data);
  AngLin6 velocities[6]{}, impulses[3]{};
  velocities[0].lin={0,-9.81f,0}; velocities[1].lin={0,-9.81f,0};
  velocities[2].lin={0,0,0}; velocities[3].lin={0,2,0};
  velocities[2].ang=velocities[3].ang={0,0,-2};
  velocities[5].lin={0,-9.81f,0};
  StressProcessor::SolverParams settings; settings.maxIter=100;
  require(processor.solve(impulses,velocities,settings)>=0,"rigid modes solve");
  for(int i=0;i<2;++i) {
    require(std::abs(impulses[i].lin.y)<1e-4f && std::abs(impulses[i].ang.z)<1e-4f,"free motion created internal load");
  }
  require(std::abs(std::abs(impulses[2].lin.y)-19.62f)<1e-3f,"anchored gravity removed incorrectly");
  // Break the anchored bond; the released node now free-falls. Projection must
  // rebuild connected components even though node masses did not change.
  require(processor.removeBond(2),"remove bond");
  settings.warmStart=false;
  require(processor.solve(impulses,velocities,settings)>=0,"projection after split");
  require(processor.equilibriumError()<1e-4f,"split equilibrium error");
  // Non-finite input must be reported as numerical failure, never converged.
  velocities[0].lin.x=std::numeric_limits<float>::quiet_NaN();
  require(processor.solve(impulses,velocities,settings)<0,"NaN reported converged");
}
}
int main() {
  try {
    rigidModesAndSplit();
    blast::BlastRuntime runtime;
    require(runtime.init(),"runtime");
    eccentricSupport(runtime,1e-6f);
    eccentricSupport(runtime,1e-3f); // loose normal residual must not bypass equilibrium verification
    require(runtime.errorCount()==0,"Blast errors");
    std::puts("OK convergence: cut balance, rigid modes, split, budget, sleep, changed load, numerical failure");
    return 0;
  } catch(const std::exception& e) { std::fprintf(stderr,"FAIL: %s\n",e.what()); return 1; }
}
