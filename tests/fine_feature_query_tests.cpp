#include "physics/FineFeatureQuery.h"
#include <iostream>
#include <random>
#include <stdexcept>

int main() {
  using namespace physics;
  try {
    std::mt19937 rng(1919);
    for (int n : {1, 16, 64, 256, 1024}) {
      std::vector<uint32_t> ids;
      for (int i=0;i<3000;++i) ids.push_back(packFine(rng()%n,rng()%n,rng()%n,n));
      std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());
      for (int q=0;q<2000;++q) {
        glm::ivec3 a(rng()%n,rng()%n,rng()%n),b(rng()%n,rng()%n,rng()%n);
        const auto mn=glm::min(a,b), mx=glm::max(a,b);
        std::vector<uint32_t> expected,actual;
        for(auto id:ids) {
          auto p=unpackFine(id,n);
          if(p.x>=mn.x&&p.y>=mn.y&&p.z>=mn.z&&p.x<=mx.x&&p.y<=mx.y&&p.z<=mx.z) expected.push_back(id);
        }
        queryFineFeatures(ids,n,mn,mx,[&](uint32_t id){actual.push_back(id);});
        if(actual!=expected) throw std::runtime_error("feature selection/order mismatch");
      }
    }
    const int n=1024;
    std::vector<uint32_t> sparse{packFine(0,0,0,n),packFine(1023,1023,1023,n)};
    int hits=0;
    int rows=queryFineFeatures(sparse,n,glm::ivec3(0),glm::ivec3(n-1),[&](uint32_t){++hits;});
    if(hits!=2||rows!=2) throw std::runtime_error("empty row skip failed");
    queryFineFeatures(sparse,n,glm::ivec3(1),glm::ivec3(0),[](uint32_t){throw std::runtime_error("invalid box");});
    std::cout<<"PASS 10000 randomized ordered feature queries; sparse full-grid row visits="<<rows
             <<" vs "<<n*n<<" dense rows\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
