
#include "../rays/bvh.h"
#include "debug.h"
#include "float.h"
#include <iostream>
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build_helper(size_t& parent_node_i, size_t l, size_t size, size_t max_leaf_size) {
  // Base case: size of sublist <= max leaf size.
  if(size <= max_leaf_size) {
    return;
  }

  // define variables for later usage
  const int NUM_BUCKETS = 16;
  float c_trav = 1;
  float c_isect = 8;

  /* Create variables to store best possible cut */
  float lowest_cost = FLT_MAX;
  std::vector<BBox> best_left;
  std::vector<BBox> best_right;
  BBox best_leftBox;
  BBox best_rightBox;
  // size_t best_i;
  size_t best_n;

  std::vector<std::vector<size_t>> best_buckets;
  
  /* Check 3 dimensions of axis */
  for (size_t i = 0; i < 3; i++) {
    float b_start = FLT_MAX;
    float b_end = FLT_MIN;
    /* Calculate b_start/b_end for specified l, r ranges. */
    for (size_t p = l; p < l + size; p++) {
      float b_curr = primitives[p].bbox().center()[i];
      if (b_curr < b_start) b_start = b_curr;
      if (b_curr > b_end) b_end = b_curr;
    }
    
    /* Determine how big to make each bucket by range along axis */
    float b_delta = (b_end - b_start) / NUM_BUCKETS;
    
    /* Assign primitives to their respective buckets */
    std::vector<std::vector<size_t>> buckets;
    std::vector<BBox> bbuckets; // BBox for each bucket
    for (float b_loc = b_start; b_loc <= b_end; b_loc += b_delta) {
        std::vector<size_t> b_prims;
        BBox bbucket = BBox();
        for (size_t m = l; m < l + size; m++) {
          float p_center = primitives[m].bbox().center()[i];
          // only add primitive to bucket if within range
          // i.e. within curr b_loc and b_loc + b_delta
          if (p_center >= b_loc && p_center < (b_loc + b_delta)) {
            if (!b_prims.size()) {bbucket = primitives[m].bbox();}
            else {bbucket.enclose(primitives[m].bbox());}
            b_prims.push_back(m);
          }
        }
        bbuckets.push_back(bbucket);
        buckets.push_back(b_prims);
    }

    /* Find lowest cost slice along buckets */
    /* BUCKETS - 1 -> num of unique partitions */
    for (float n = 1; n < NUM_BUCKETS; n++) {
      std::vector<BBox> left_bbuckets(bbuckets.begin(), bbuckets.begin() + n);
      std::vector<BBox> right_bbuckets(bbuckets.begin() + n, bbuckets.end());

      float n_a = left_bbuckets.size();
      float n_b = right_bbuckets.size();

      BBox b_a;
      for (size_t j = 0; j < left_bbuckets.size(); j++) {
        if (j == 0) {b_a = left_bbuckets[j];}
        else {b_a.enclose(left_bbuckets[j]);}
      }
      float s_a = b_a.surface_area();

      BBox b_b;
      for (size_t k = 0; k < right_bbuckets.size(); k++) {
        if (k == 0) {b_b = right_bbuckets[k];}
        else { b_b.enclose(right_bbuckets[k]);}
      }
      float s_b = b_b.surface_area();

      float cost = c_trav
                    + (s_a / nodes[parent_node_i].bbox.surface_area()) * n_a * c_isect
                    + (s_b / nodes[parent_node_i].bbox.surface_area()) * n_b * c_isect;

      if (cost < lowest_cost) {
          lowest_cost = cost;
          best_left = left_bbuckets;
          best_right = right_bbuckets;
          best_leftBox = b_a;
          best_rightBox = b_b;
          best_n = n;
          // best_i = i;
          best_buckets = buckets;
      }
    }
  }

  // Note that by construction in this simple example, the primitives are
  // contiguous as required. But in the students real code, students are
  // responsible for reorganizing the primitives in the primitives array so that
  // after a SAH split is computed, the chidren refer to contiguous ranges of primitives.
  
  /* Final vector to dictate grouping of Primitives. */
  std::vector<size_t> final_left;
  std::vector<size_t> final_right;

  for (size_t i = 0; i < best_buckets.size(); i++) {
    /* Add to respective final */
    for (size_t j = 0; j < best_buckets[i].size(); j++) {
      (i < best_n) ?
        final_left.push_back(best_buckets[i][j]):
        final_right.push_back(best_buckets[i][j]);
    }
  }

  /* Check if there has been a split - neither final_left & final_right
     are equal to param size. */
  /* TODO: Complete splitting and recalculating bboxes. */
  if (final_left.size() == size || final_right.size() == size) {
    std::cout << "No actual split. Exiting recursion..." << std::endl;
    return;
  }

  /* Swap primitives inside of primitives to maintain contiguous ranges.
     Not guaranteed that final_left/final_right are of equal size.
     Update separately. */
  for (size_t i = l; i < l + final_left.size(); i++) {
    /* Update value of parent_node_i as needed. */
    // if (i == parent_node_i) parent_node_i = final_left[i - l];
    std::swap(primitives[i], primitives[final_left[i - l]]);
  }
  for (size_t i = l + final_left.size(); i < l + final_left.size() + final_right.size(); i++) {
    /* Update value of parent_node_i as needed. */
    // if (i == parent_node_i) parent_node_i = final_right[i - l - final_left.size()];
    std::swap(primitives[i], primitives[final_right[i - l - final_left.size()]]);
  }

  size_t startl = l;
  size_t rangel = final_left.size();

  size_t startr = l + final_left.size();
  size_t ranger = final_right.size();

  // create child nodes
  size_t node_addr_l = new_node();
  size_t node_addr_r = new_node();
  nodes[parent_node_i].l = node_addr_l;
  nodes[parent_node_i].r = node_addr_r;

  nodes[node_addr_l].bbox = best_leftBox;
  nodes[node_addr_l].start = startl;
  nodes[node_addr_l].size = rangel;

  nodes[node_addr_r].bbox = best_rightBox;
  nodes[node_addr_r].start = startr;
  nodes[node_addr_r].size = ranger;

  std::cout << "Calling helper with root " << node_addr_l << ", start " << startl << ", size " << rangel << ", and max_leaf_size " << max_leaf_size << std::endl;
  build_helper(node_addr_l, startl, rangel, max_leaf_size);
  std::cout << "Calling helper with root " << node_addr_r << ", start " << startr << ", size " << ranger << ", and max_leaf_size " << max_leaf_size << std::endl;
  build_helper(node_addr_r, startr, ranger, max_leaf_size);
} 

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    // compute bounding box for all primitives
    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    Node& node = nodes[root_node_addr];
    node.bbox = bb;
    node.start = 0;
    node.size = primitives.size();

    /* TODO: Remove before submission. Used for testing other tasks ONLY. */
    // Create bounding boxes for children
    BBox split_leftBox;
    BBox split_rightBox;

    // compute bbox for left child
    Primitive& p = primitives[0];
    BBox pbb = p.bbox();
    split_leftBox.enclose(pbb);

    // compute bbox for right child
    for(size_t i = 1; i < primitives.size(); ++i) {
        Primitive& p = primitives[i];
        BBox pbb = p.bbox();
        split_rightBox.enclose(pbb);
    }

    // Note that by construction in this simple example, the primitives are
    // contiguous as required. But in the students real code, students are
    // responsible for reorganizing the primitives in the primitives array so that
    // after a SAH split is computed, the chidren refer to contiguous ranges of primitives.

    size_t startl = 0;  // starting prim index of left child
    size_t rangel = 1;  // number of prims in left child
    size_t startr = startl + rangel;  // starting prim index of right child
    size_t ranger = primitives.size() - rangel; // number of prims in right child

    // create child nodes
    size_t node_addr_l = new_node();
    size_t node_addr_r = new_node();
    nodes[root_node_addr].l = node_addr_l;
    nodes[root_node_addr].r = node_addr_r;

    nodes[node_addr_l].bbox = split_leftBox;
    nodes[node_addr_l].start = startl;
    nodes[node_addr_l].size = rangel;

    nodes[node_addr_r].bbox = split_rightBox;
    nodes[node_addr_r].start = startr;
    nodes[node_addr_r].size = ranger;

    /* Use helper function to track current range of focus. */
    // std::cout << "Calling helper with root " << root_node_addr << ", start 0, size " << node.size << ", and max_leaf_size " << max_leaf_size << std::endl;
    // build_helper(root_node_addr, 0, node.size, max_leaf_size);
    // std::cout << "BVH Tree built. Exiting..." <<std::endl;
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive>
BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive>
bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive>
BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive>
std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive>
void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
