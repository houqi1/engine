#include "physics/DynamicAabbTree.h"
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace physics;
void require(bool b) { if (!b) throw std::runtime_error("broad phase oracle mismatch"); }
int main() {
  try {
    DynamicAabbTree tree;
    constexpr int count = 1024;
    std::vector<int> proxies(count, -1);
    std::vector<PhysicsAabb> boxes(count);
    std::mt19937 rng(7619);
    std::uniform_real_distribution<float> pos(-80,80), extent(0.01f,8);
    auto randomBox = [&]() {
      glm::vec3 p(pos(rng),pos(rng),pos(rng));
      return PhysicsAabb{p,p+glm::vec3(extent(rng),extent(rng),extent(rng))};
    };
    for (int i=0;i<count;++i) {
      boxes[i]=randomBox(); proxies[i]=tree.insert(i,boxes[i]);
    }
    // Oracle covers insert, delete, reused object slots, large teleports,
    // shrink/grow edits and small movements retained inside fat bounds.
    for (int step=0;step<12000;++step) {
      int id=static_cast<int>(rng()%count);
      if (step%7==0) {
        if(proxies[id]!=-1) tree.remove(proxies[id]);
        proxies[id]=-1;
      } else {
        if(step%3==0) { boxes[id].min+=glm::vec3(.01f); boxes[id].max+=glm::vec3(.01f); }
        else boxes[id]=randomBox();
        if(proxies[id]==-1) proxies[id]=tree.insert(id,boxes[id]);
        else tree.update(proxies[id],boxes[id],.2f);
      }
      PhysicsAabb q=step%2?randomBox():boxes[id];
      std::vector<bool> found(count,false);
      tree.query(q,[&](int j){require(proxies[j]!=-1&&!found[j]);found[j]=true;});
      for(int j=0;j<count;++j)
        if(proxies[j]!=-1&&boxes[j].overlaps(q)) require(found[j]);
      require(tree.height()<24);
    }
    for(int id:proxies) if(id!=-1) tree.remove(id);
    require(tree.query({glm::vec3(-1000),glm::vec3(1000)},[](int){require(false);})==0);
    tree.clear();
    // Sorted insertion must not degenerate; measure work rather than fragile time.
    for(int i=0;i<count;++i) tree.insert(i,{glm::vec3(i*4.0f,0,0),glm::vec3(i*4.0f+1,1,1)});
    int visits=0,hits=0;
    for(int i=0;i<count;++i)
      visits+=tree.query({glm::vec3(i*4.0f,0,0),glm::vec3(i*4.0f+1,1,1)},[&](int j){require(j==i);++hits;});
    require(hits==count&&visits<count*40&&tree.height()<24);
    std::cout<<"PASS randomized updates/removal/reuse, conservative queries, sorted insertion\n"
             <<"1024 separated objects: "<<visits<<" node tests vs "<<count*(count-1)/2
             <<" unordered brute-force pairs; height="<<tree.height()<<"\n";
    // Unequal voxel sizes: broad-phase padding must include the old pair gate.
    for(int i=0;i<10000;++i) {
      auto a=randomBox(),b=randomBox(); const float sa=extent(rng),sb=extent(rng);
      if(a.expanded(std::max(sa,sb)).overlaps(b.expanded(std::max(sa,sb))))
        require(a.expanded(2*sa).overlaps(b.expanded(2*sb)));
    }
    // Boundary contact is inclusive, and an empty/reset tree is reusable.
    require(PhysicsAabb{glm::vec3(0),glm::vec3(1)}.overlaps({glm::vec3(1),glm::vec3(2)}));
    std::cout<<"PASS mixed voxel scales and touching boundaries\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
