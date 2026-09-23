#pragma once

#include <glm/glm.hpp>
#include <algorithm>
#include <vector>

namespace physics {

struct PhysicsAabb {
  glm::vec3 min{0.0f}, max{0.0f};
  bool overlaps(const PhysicsAabb& b) const {
    return min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y &&
           max.y >= b.min.y && min.z <= b.max.z && max.z >= b.min.z;
  }
  bool contains(const PhysicsAabb& b) const {
    return min.x <= b.min.x && min.y <= b.min.y && min.z <= b.min.z &&
           max.x >= b.max.x && max.y >= b.max.y && max.z >= b.max.z;
  }
  PhysicsAabb expanded(float margin) const { return {min - glm::vec3(margin), max + glm::vec3(margin)}; }
  static PhysicsAabb unite(const PhysicsAabb& a, const PhysicsAabb& b) {
    return {glm::min(a.min, b.min), glm::max(a.max, b.max)};
  }
  float area() const {
    const glm::vec3 d = max - min;
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
  }
};

// Persistent, height-balanced binary BVH. Proxies are stable node indices until
// removed. Small movements inside a fat bound do not change the tree topology.
class DynamicAabbTree {
public:
  void clear() { nodes_.clear(); free_ = root_ = -1; }
  int insert(int object, const PhysicsAabb& bounds) {
    const int leaf = allocate();
    nodes_[leaf].box = bounds;
    nodes_[leaf].object = object;
    attach(leaf);
    return leaf;
  }
  void remove(int leaf) { detach(leaf); release(leaf); }
  bool update(int leaf, const PhysicsAabb& bounds, float margin) {
    // Shrinking edits/slot reuse should not retain arbitrarily large old boxes.
    if (nodes_[leaf].box.contains(bounds) && bounds.expanded(4.0f * margin).contains(nodes_[leaf].box))
      return false;
    detach(leaf);
    nodes_[leaf].box = bounds.expanded(margin);
    attach(leaf);
    return true;
  }
  template<class Visitor> int query(const PhysicsAabb& bounds, Visitor&& visitor) const {
    int visits = 0;
    queryNode(root_, bounds, visitor, visits);
    return visits;
  }
  int height() const { return root_ == -1 ? 0 : nodes_[root_].height; }

private:
  struct Node {
    PhysicsAabb box;
    int parent = -1, left = -1, right = -1, height = 0, object = -1;
    bool leaf() const { return left == -1; }
  };
  std::vector<Node> nodes_;
  int root_ = -1, free_ = -1;
  int allocate() {
    if (free_ == -1) { nodes_.emplace_back(); return static_cast<int>(nodes_.size()) - 1; }
    const int id = free_;
    free_ = nodes_[id].parent;
    nodes_[id] = Node{};
    return id;
  }
  void release(int id) { nodes_[id].parent = free_; free_ = id; }
  void refit(int id) {
    Node& n = nodes_[id];
    n.box = PhysicsAabb::unite(nodes_[n.left].box, nodes_[n.right].box);
    n.height = 1 + std::max(nodes_[n.left].height, nodes_[n.right].height);
  }
  void replaceChild(int parent, int oldChild, int child) {
    if (parent == -1) root_ = child;
    else if (nodes_[parent].left == oldChild) nodes_[parent].left = child;
    else nodes_[parent].right = child;
    if (child != -1) nodes_[child].parent = parent;
  }
  int rotateLeft(int a) {
    const int b = nodes_[a].right, middle = nodes_[b].left;
    replaceChild(nodes_[a].parent, a, b);
    nodes_[b].left = a; nodes_[a].parent = b;
    nodes_[a].right = middle; nodes_[middle].parent = a;
    refit(a); refit(b); return b;
  }
  int rotateRight(int a) {
    const int b = nodes_[a].left, middle = nodes_[b].right;
    replaceChild(nodes_[a].parent, a, b);
    nodes_[b].right = a; nodes_[a].parent = b;
    nodes_[a].left = middle; nodes_[middle].parent = a;
    refit(a); refit(b); return b;
  }
  void repair(int id) {
    while (id != -1) {
      refit(id);
      const int l = nodes_[id].left, r = nodes_[id].right;
      if (nodes_[r].height - nodes_[l].height > 1) {
        if (nodes_[nodes_[r].left].height > nodes_[nodes_[r].right].height) rotateRight(r);
        id = rotateLeft(id);
      } else if (nodes_[l].height - nodes_[r].height > 1) {
        if (nodes_[nodes_[l].right].height > nodes_[nodes_[l].left].height) rotateLeft(l);
        id = rotateRight(id);
      }
      id = nodes_[id].parent;
    }
  }
  void attach(int leaf) {
    if (root_ == -1) { root_ = leaf; nodes_[leaf].parent = -1; return; }
    int sibling = root_;
    while (!nodes_[sibling].leaf()) {
      const int l = nodes_[sibling].left, r = nodes_[sibling].right;
      const float lc = PhysicsAabb::unite(nodes_[l].box, nodes_[leaf].box).area() - nodes_[l].box.area();
      const float rc = PhysicsAabb::unite(nodes_[r].box, nodes_[leaf].box).area() - nodes_[r].box.area();
      sibling = lc < rc || (lc == rc && nodes_[l].height <= nodes_[r].height) ? l : r;
    }
    const int oldParent = nodes_[sibling].parent;
    const int parent = allocate();
    replaceChild(oldParent, sibling, parent);
    nodes_[parent].left = sibling; nodes_[parent].right = leaf;
    nodes_[sibling].parent = parent; nodes_[leaf].parent = parent;
    repair(parent);
  }
  void detach(int leaf) {
    if (leaf == root_) { root_ = -1; return; }
    const int parent = nodes_[leaf].parent, grand = nodes_[parent].parent;
    const int sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
    replaceChild(grand, parent, sibling);
    release(parent);
    repair(grand);
  }
  template<class Visitor> void queryNode(int id, const PhysicsAabb& box, Visitor& visitor, int& visits) const {
    if (id == -1) return;
    ++visits;
    const Node& n = nodes_[id];
    if (!n.box.overlaps(box)) return;
    if (n.leaf()) visitor(n.object);
    else { queryNode(n.left, box, visitor, visits); queryNode(n.right, box, visitor, visits); }
  }
};
} // namespace physics
