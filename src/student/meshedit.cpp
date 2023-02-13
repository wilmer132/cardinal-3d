
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

using namespace std;

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    /* Center is new location. */
    Vec3 center = e->center();

    /* Shift first vertex towards the center. */
    HalfedgeRef edge_he = e->halfedge();
    VertexRef centered_v = edge_he->vertex();
    centered_v->pos = center;

    /* Get information for opposite side / vertex. */
    HalfedgeRef edge_he_twin = edge_he->twin();
    VertexRef opposite_v = edge_he_twin->vertex();
    HalfedgeRef opposite_v_he = opposite_v->halfedge();

    /* Get first two faces to save for later. */
    FaceRef top_face = edge_he->face();
    FaceRef bottom_face = edge_he_twin->face();

    /* Get half-edges from faces that we will delete later. */
    std::set<HalfedgeRef> top_face_halfedges;
    HalfedgeRef starting_he_top = top_face->halfedge();
    HalfedgeRef curr_he_top = starting_he_top;
    do {
        top_face_halfedges.insert(curr_he_top);
        curr_he_top = curr_he_top->next();
    } while(curr_he_top != starting_he_top);

    std::set<HalfedgeRef> bottom_face_halfedges;
    HalfedgeRef starting_he_bot = bottom_face->halfedge();
    HalfedgeRef curr_he_bot = starting_he_bot;
    do {
        bottom_face_halfedges.insert(curr_he_bot);
        curr_he_bot = curr_he_bot->next();
    } while(curr_he_bot != starting_he_bot);

    /* Get twins that we will need to rewire later. */
    bool have_zero_area = (top_face_halfedges.size() == 3);
    vector<HalfedgeRef> top_twins;
    EdgeRef top_edge_to_delete, top_edge_to_keep;
    vector<HalfedgeRef> bottom_twins;
    EdgeRef bot_edge_to_delete, bot_edge_to_keep;
    if(have_zero_area) {
        top_twins.push_back(edge_he->next()->twin());
        top_twins.push_back(edge_he->next()->next()->twin());
        top_edge_to_keep = edge_he->next()->twin()->edge();
        top_edge_to_delete = edge_he->next()->next()->twin()->edge();

        bottom_twins.push_back(edge_he_twin->next()->twin());
        bottom_twins.push_back(edge_he_twin->next()->next()->twin());
        bot_edge_to_keep = edge_he_twin->next()->twin()->edge();
        bot_edge_to_delete = edge_he_twin->next()->next()->twin()->edge();
    }

    /* Move half-edges that are pointing at second vertex to point to center spot. */
    HalfedgeRef start_he_shift = opposite_v->halfedge();
    HalfedgeRef curr_he_shift = start_he_shift;
    do {
        HalfedgeRef temp = curr_he_shift->twin()->next();
        curr_he_shift->_vertex = centered_v;
        curr_he_shift = temp;
    } while(curr_he_shift != start_he_shift);

    centered_v->_halfedge = opposite_v_he;

    /* If there is a zero area face, rewire and erase appropriately. */
    if (have_zero_area) {
        HalfedgeRef adj_he_top = top_twins[0];
        HalfedgeRef opp_he_top = top_twins[1];
        adj_he_top->set_neighbors(adj_he_top->next(), opp_he_top, adj_he_top->vertex(), top_edge_to_keep, adj_he_top->face());
        adj_he_top->vertex()->_halfedge = adj_he_top;
        opp_he_top->set_neighbors(opp_he_top->next(), adj_he_top, centered_v, top_edge_to_keep, opp_he_top->face());
        top_edge_to_keep->_halfedge = adj_he_top;
        
        HalfedgeRef adj_he_bot = bottom_twins[0];
        HalfedgeRef opp_he_bot = bottom_twins[1];
        adj_he_bot->set_neighbors(adj_he_bot->next(), opp_he_bot, adj_he_bot->vertex(), bot_edge_to_keep, adj_he_bot->face());
        adj_he_bot->vertex()->_halfedge = adj_he_bot;
        opp_he_bot->set_neighbors(opp_he_bot->next(), adj_he_bot, centered_v, bot_edge_to_keep, opp_he_bot->face());
        bot_edge_to_keep->_halfedge = adj_he_bot;

        erase(top_face);
        for(HalfedgeRef h : top_face_halfedges) {
            erase(h);
        }

        erase(bottom_face);
        for(HalfedgeRef h : bottom_face_halfedges) {
            erase(h);
        }

        erase(top_edge_to_delete);
        erase(bot_edge_to_delete);
    }

    erase(e);
    erase(opposite_v);

    validate();

    return centered_v;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    /* Record edges to change: 
       h - half-edge
       h_p - half-edge prime
       m - medium half edge
       m_p - same as before
       n - next half-edge 
       n_p - same as before. */
    HalfedgeRef h = e->halfedge();
    HalfedgeRef h_p = h->twin();
    HalfedgeRef m = h->next();
    HalfedgeRef m_p = h_p->next();
    HalfedgeRef n = m->next();
    HalfedgeRef n_p = m_p->next();

    /* Grab vertices from half-edge and prime. */
    VertexRef h_v = h->vertex();
    VertexRef h_p_v = h_p->vertex();

    /* Update old half edge/twin. */
    h->set_neighbors(n, h_p, n_p->vertex(), e, h->face());
    h->vertex()->_halfedge = h;
    h_p->set_neighbors(n_p, h, n->vertex(), h_p->edge(), h_p->face());
    h_p->vertex()->_halfedge = h_p;

    /* Update middle half edge/twin. */
    m->set_neighbors(h_p, m->twin(), h_p_v, m->edge(), h_p->face());
    m->vertex()->_halfedge = m;
    m->face()->_halfedge = m;
    m_p->set_neighbors(h, m_p->twin(), h_v, m_p->edge(), h->face());
    m_p->vertex()->_halfedge = m_p;
    m_p->face()->_halfedge = m_p;

    /* Retrieve last half edges. Update them. */
    while(n->next() != h) n = n->next();
    n->set_neighbors(m_p, n->twin(), n->vertex(), n->edge(), n->face());
    while(n_p->next() != h_p) n_p = n_p->next();
    n_p->set_neighbors(m, n_p->twin(), n_p->vertex(), n_p->edge(), n_p->face());

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
  /* Assume triangle polygons only. Do nothing, otherwise. */
  if (e->halfedge()->face()->degree() > 3) return std::nullopt;

  /* Retrieve vertex coordinates. E.g. up, down, right, left, etc. */
  VertexRef v = e->halfedge()->vertex();
  VertexRef v_c = new_vertex();
  VertexRef v_r = e->halfedge()->next()->next()->vertex();
  VertexRef v_d = e->halfedge()->twin()->vertex();
  VertexRef v_l = e->halfedge()->twin()->next()->next()->vertex();
  
  /* Create three inner edges. Recycle existing edge. 
     i.e. edge-right, -down, -low, etc. */
  EdgeRef e_r = new_edge();
  EdgeRef e_d = new_edge();
  EdgeRef e_l = new_edge();

  /* Create six half-edges. Recycle existing half-edges. */
  HalfedgeRef h = e->halfedge()->twin(); // NOTE: Twin
  HalfedgeRef h_t = e->halfedge();
  HalfedgeRef h_r = new_halfedge();
  HalfedgeRef h_r_t = new_halfedge();
  HalfedgeRef h_d = new_halfedge();
  HalfedgeRef h_d_t = new_halfedge();
  HalfedgeRef h_l = new_halfedge();
  HalfedgeRef h_l_t = new_halfedge();

  /* Retrieve relevant existing half-edges. */
  HalfedgeRef h_ur = h_t->next()->next();
  HalfedgeRef h_lr = h_t->next();
  HalfedgeRef h_ul = h->next();
  HalfedgeRef h_ll = h->next()->next();

  /* Create two faces. Recycle existing faces. */
  FaceRef f_ur = e->halfedge()->face();
  FaceRef f_lr = new_face();
  FaceRef f_ll = e->halfedge()->twin()->face();
  FaceRef f_ul = new_face();

  /* Adjust middle vertex position. */
  v_c->pos = e->center();

  /* Set up half edge's neighbors. */
  h->set_neighbors(h_ul, h_t, v_c, e, f_ul);
  h_t->set_neighbors(h_r, h, v, e, f_ur);
  h_r->set_neighbors(h_ur, h_r_t, v_c, e_r, f_ur);
  h_r_t->set_neighbors(h_d, h_r, v_r, e_r, f_lr);
  h_d->set_neighbors(h_lr, h_d_t, v_c, e_d, f_lr);
  h_d_t->set_neighbors(h_l, h_d, v_d, e_d, f_ll);
  h_l->set_neighbors(h_ll, h_l_t, v_c, e_l, f_ll);
  h_l_t->set_neighbors(h, h_l, v_l, e_l, f_ul);

  /* Set up older half edges's neighbors. */
  h_ur->set_neighbors(h_t, h_ur->twin(), h_ur->vertex(), h_ur->edge(), f_ur);
  h_lr->set_neighbors(h_r_t, h_lr->twin(), h_lr->vertex(), h_lr->edge(), f_lr);
  h_ll->set_neighbors(h_d_t, h_ll->twin(), h_ll->vertex(), h_ll->edge(), f_ll);
  h_ul->set_neighbors(h_l_t, h_ul->twin(), h_ul->vertex(), h_ul->edge(), f_ul);

  /* Set up vertices. */
  v->_halfedge = h_t;
  v_c->_halfedge = h_d;
  v_r->_halfedge = h_r_t;
  v_d->_halfedge = h_d_t;
  v_l->_halfedge = h_l_t;

  /* Set up inner edges. */
  e->_halfedge = h;
  e_r->_halfedge = h_r;
  e_d->_halfedge = h_d;
  e_l->_halfedge = h_l;

  /* Set up faces. */
  f_ur->_halfedge = h_t;
  f_lr->_halfedge = h_r_t;
  f_ll->_halfedge = h_d_t;
  f_ul->_halfedge = h_l_t;

  return v_c;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determines the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)f;
    return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/

void Halfedge_Mesh::triangulate() {
    /* Loop through all of the faces in the shape. */
    /* Initialize start and end so we don't end up in infinite loop as we add more faces. */
    FaceRef first_face = faces_begin();
    FaceRef last_face = faces_end();
    for (FaceRef f = first_face; f != last_face; f++) {
        /* Starting positions to remember as we traverse.*/
        HalfedgeRef start_he = f->halfedge();
        VertexRef start_v = start_he->vertex();

        /* Halfedges to keep track of the sides that we have already encountered. */
        HalfedgeRef opp_side = start_he;
        HalfedgeRef adj_side = start_he->next();

        /* Start with the half-edge that is two edges away. */
        HalfedgeRef curr_he = start_he->next()->next();

        /* Do not do all the work for a side that is already triangular. */
        if (curr_he->next() == start_he) {
            continue;
        }

        /* Traverse all sides, and draw an edge when needed. */
        do {
            /* We have found a loop inside our triangle. */
            if (curr_he->next() == opp_side) {
                break;
            }
            /* Create variables that represent new edge and new face. */
            EdgeRef edge_new = new_edge();
            FaceRef face_new = new_face();

            /* Halfedges that go from current vertex to starting vertex. */
            HalfedgeRef curr_to_start_he = new_halfedge();
            HalfedgeRef start_to_curr_he = new_halfedge();

            VertexRef curr_v = curr_he->vertex();

            /* Adjust new edge across. */
            curr_to_start_he->set_neighbors(opp_side, start_to_curr_he, curr_v, edge_new, face_new);
            start_to_curr_he->set_neighbors(curr_he, curr_to_start_he, start_v, edge_new, f);

            /* Adjust our additional sides that will now connect inside triangle. */
            adj_side->set_neighbors(curr_to_start_he, adj_side->twin(), adj_side->vertex(), adj_side->edge(), face_new);
            opp_side->set_neighbors(adj_side, opp_side->twin(), opp_side->vertex(), opp_side->edge(), face_new);

            /* Adjust faces and new edge. */
            face_new->halfedge() = curr_to_start_he;
            edge_new->halfedge() = curr_to_start_he;

            /* Update variables to shift over as we move around loop.*/
            opp_side = start_to_curr_he;
            adj_side = curr_he;
            curr_he = curr_he->next();

            /* For final triangle, make sure to rewire all three sides. */
            if (curr_he->next() == start_he) { 
                adj_side->set_neighbors(curr_he, adj_side->twin(), adj_side->vertex(), adj_side->edge(), f);
                opp_side->set_neighbors(adj_side, curr_to_start_he, start_v, edge_new, f);
                curr_he->set_neighbors(opp_side, curr_he->twin(), curr_he->vertex(), curr_he->edge(), f);
                f->halfedge() = curr_he;
            }
        } while (curr_he != start_he);
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
