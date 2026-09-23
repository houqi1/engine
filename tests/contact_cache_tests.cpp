#include "physics/ContactCache.h"
#include "physics/Solver.h"
#include <iostream>
#include <stdexcept>

using namespace physics;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
bool near(float a,float b,float e=1e-4f) { return std::abs(a-b)<e; }

struct Fixture {
  ContactCache cache;
  RigidBody a,b;
  ContactBodyStamp sa{1,1,.1f},sb{1,1,.1f};
  std::vector<Contact> contacts;
  int generated=0;
  bool empty=false, reverse=false;
  Fixture() {
    a.shapeIndex=0; a.dynamic=a.awake=true; a.invM=1; a.extent=.5f;
    a.Iloc=glm::mat3(1); refreshInverseInertiaWorld(a);
    b.shapeIndex=1;
  }
  ContactCache::Stats collect(int substep) {
    contacts.clear();cache.beginSubstep();
    auto stats=cache.collect(a,b,sa,sb,substep,contacts,[&](std::vector<Contact>& out){
      ++generated;if(empty)return;
      Contact c;c.a=reverse?1:0;c.b=reverse?0:1;c.fineA=10;c.fineB=20;
      c.n=glm::vec3(0,reverse?-1.0f:1.0f,0);c.d=0;
      c.p=a.x;c.rA=c.p-(reverse?b.x:a.x);c.rB=c.p-(reverse?a.x:b.x);
      out.push_back(c);
    });
    cache.endSubstep();return stats;
  }
  void solve() {
    std::vector<RigidBody> bodies{a,b};solveContacts(bodies,contacts,kSubDt);
    a=bodies[0];b=bodies[1];cache.store(contacts);
  }
};

int main() {
 try {
  Fixture f;
  f.collect(0);f.contacts[0].lambdaNVel=.2f;f.contacts[0].lambdaN=10;f.cache.store(f.contacts);
  f.a.x.y=-.005f;
  auto s=f.collect(1);
  check(s.reused==1&&near(f.contacts[0].d,.005f),"anchor depth must track motion");
  check(near(f.contacts[0].lambdaN,.2f),"warm seed must exclude Baumgarte");
  f.collect(0);check(f.generated==2,"new tick must refresh geometry");
  f.sa.revision++;f.collect(1);check(f.generated==3&&f.contacts[0].lambdaN==0,"edit invalidates seed");
  f.sa.generation++;f.collect(2);check(f.generated==4,"slot reuse invalidates cache");
  f.a.x.x+=1;f.collect(3);check(f.generated==5,"teleport requires narrow phase");
  f.a.v.x=50;f.collect(4);check(f.generated==6,"fast movement requires narrow phase");
  f.a.v=glm::vec3(0);f.a.q=glm::angleAxis(.1f,glm::vec3(0,0,1));
  f.collect(5);check(f.generated==7,"rotation requires narrow phase");
  f.cache.invalidate(0);f.collect(1);check(f.generated==8,"explicit replacement invalidates cache");
  f.a.awake=false;f.collect(2);check(f.generated==9,"sleep transition invalidates seed");
  f.cache.beginSubstep();f.cache.endSubstep();check(f.cache.size()==0,"unseen pairs must expire");
  Fixture e;e.empty=true;e.collect(0);e.empty=false;e.collect(1);
  check(e.generated==2&&e.contacts.size()==1,"empty cache must discover new contact");
  Fixture r;r.reverse=true;r.collect(0);r.a.x.y=-.004f;r.collect(1);
  check(near(r.contacts[0].d,.004f),"reversed narrow-phase orientation anchors");

  // Sustained gravity: warm starts must not add previous-step impulses to the
  // reported load again. The physical result is exactly m*g*h per substep.
  Fixture resting;float total=0;int reuse=0,warm=0;
  for(int step=0;step<600;++step) {
    resting.a.v+=kGravity*kSubDt;
    auto stats=resting.collect(step%kSubsteps);reuse+=stats.reused;warm+=stats.warmPoints;
    resting.solve();total+=contactImpulseOnA(resting.contacts[0]).y;
    check(glm::length(resting.a.v)<1e-4f,"resting support must not inject velocity");
  }
  check(near(total,600*9.81f*kSubDt,.002f),"cached impulse must be counted once per substep");
  check(resting.generated==100&&reuse==500&&warm==599,"stable contact must reuse five of six substeps");
  resting.a.v=glm::vec3(0,10,0);resting.collect(0);resting.solve();
  check(near(resting.contacts[0].lambdaN,0)&&near(resting.contacts[0].lambdaNVel,0),"separation must cancel warm support");

  // Sliding direction reversal must stay in the Coulomb disk and never speed up
  // the lateral motion. Reuse last frame's friction in world space.
  Fixture friction;friction.a.v=glm::vec3(2,-1,0);friction.collect(0);friction.solve();
  friction.a.v=glm::vec3(-2,-1,0);friction.collect(0);friction.solve();
  const Contact& c=friction.contacts[0];
  check(friction.a.v.x>=-2&&friction.a.v.x<=0,"friction reversal must oppose sliding");
  check(glm::length(c.JtWorld)<=kFriction*c.lambdaN+1e-5f,"Coulomb friction limit");
  check(std::abs(glm::dot(c.JtWorld,c.n))<1e-5f,"friction stays tangent");
  Fixture penetration;penetration.collect(0);penetration.contacts[0].d=.04f;
  penetration.solve();
  check(penetration.contacts[0].lambdaN>0&&near(penetration.contacts[0].lambdaNVel,0),
        "stationary penetration correction is not an impact load");
  penetration.a.v=glm::vec3(0);penetration.collect(0);
  check(near(penetration.contacts[0].lambdaN,0),"penetration bias must not persist into warm start");

  // Eight-body vertical contact chain. Translation-only fixture isolates load
  // transfer through multiple constraints; geometry is recomputed at tick start.
  ContactCache stackCache;
  std::vector<RigidBody> stack(9);
  for (int i=0;i<9;++i) {
    stack[i].shapeIndex=i;stack[i].x.y=static_cast<float>(i);
    stack[i].extent=.5f;
    if(i) { stack[i].awake=stack[i].dynamic=true;stack[i].invM=1; }
  }
  for(int step=0;step<1800;++step) {
    for(int i=1;i<9;++i) stack[i].v+=kGravity*kSubDt;
    std::vector<Contact> chain;
    stackCache.beginSubstep();
    for(int i=1;i<9;++i) {
      stackCache.collect(stack[i-1],stack[i],{1,1,.1f},{1,1,.1f},step%6,chain,
        [&](std::vector<Contact>& out){
          Contact contact;contact.a=i;contact.b=i-1;contact.n=glm::vec3(0,1,0);
          contact.p=(stack[i].x+stack[i-1].x)*.5f;
          contact.rA=contact.p-stack[i].x;contact.rB=contact.p-stack[i-1].x;
          contact.d=1-(stack[i].x.y-stack[i-1].x.y);out.push_back(contact);
        });
    }
    stackCache.endSubstep();solveContacts(stack,chain,kSubDt);stackCache.store(chain);
    for(int i=1;i<9;++i) stack[i].x+=stack[i].v*kSubDt;
  }
  check(std::abs(stack[8].x.y-8)<.08f&&glm::length(stack[8].v)<.05f,
        "contact chain must settle without sinking or jitter");
  std::cout<<"PASS eight-body support chain: top="<<stack[8].x.y<<" speed="<<glm::length(stack[8].v)<<'\n';
  std::cout<<"PASS cache lifetime/identity, anchors, fast motion, wake/edit invalidation\n"
           <<"PASS load accounting, release, friction reversal\n"
           <<"600 substeps: narrow="<<resting.generated<<" (includes release), reused="<<reuse<<", warm="<<warm<<"\n";
  return 0;
 } catch(const std::exception& e) { std::cerr<<"FAIL: "<<e.what()<<'\n';return 1; }
}
