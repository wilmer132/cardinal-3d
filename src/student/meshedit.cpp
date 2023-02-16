
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
  /* Define variables for implementation*/
  VertexRef v = e->halfedge()->vertex(); /* Main vertex. */
  VertexRef v_t = e->halfedge()->twin()->vertex(); /* Twin vertex. */
  FaceRef f = e->halfedge()->face(); /* Edge face. */
  FaceRef f_t = e->halfedge()->twin()->face(); /* Twin face. */

  /* Shift v to center. */
  v->pos = e->center();

  /* Collect face half-edges for deletion later. */
  vector<HalfedgeRef> f_he;
  vector<HalfedgeRef> f_t_he;
  HalfedgeRef h_c; /*half-edge connect. */
  HalfedgeRef h_t_c; /*half-edge connect. */
  HalfedgeRef h_i = f->halfedge(); /* Iterator half-edge. */
  do {
    f_he.push_back(h_i);
    if (h_i->next() == e->halfedge()) h_c = h_i;
    h_i = h_i->next();
  } while (h_i != f->halfedge());

  h_i = f_t->halfedge();
  do {
    f_t_he.push_back(h_i);
    if (h_i->next() == e->halfedge()->twin()) h_t_c = h_i;
    h_i = h_i->next();
  } while (h_i != f_t->halfedge());

  /* Half-edges pointing at v_t to point to v. */
  h_i = e->halfedge()->twin();
  do {
    h_i->_vertex = v;
    h_i = h_i->twin()->next();
  } while (h_i != e->halfedge()->twin());

  /* Assign v's half edge to its next vertex. */ /*Assumes half-edge will NOT be deleted. */
  v->_halfedge = e->halfedge()->twin()->next();

  /* Collapse of face with degree 3 must be erased. */
  if (f_he.size() == 3) {

    HalfedgeRef s_h_t = e->halfedge()->next()->twin(); /*second inner half-edge twin. */
    HalfedgeRef l_h_t = e->halfedge()->next()->next()->twin(); /*last inner half-edge twin. */
    EdgeRef s_e = s_h_t->edge();

    /*Adjust half-egdes prior to erasing. */
    l_h_t->set_neighbors(l_h_t->next(), s_h_t, l_h_t->vertex(), l_h_t->edge(), l_h_t->face());
    l_h_t->vertex()->_halfedge = l_h_t;
    l_h_t->edge()->_halfedge = l_h_t;
    s_h_t->set_neighbors(s_h_t->next(), l_h_t, s_h_t->vertex(), l_h_t->edge(), s_h_t->face());
    s_h_t->vertex()->_halfedge = s_h_t;
    s_h_t->edge()->_halfedge = s_h_t;

    /*Delete second edge*/
    erase(s_e);
    /*Delete face*/
    erase(f);
    /*Delete inner half edges except main. */
    h_i = e->halfedge(); /*first inner half-edge. */
    do {
      HalfedgeRef to_del = h_i;
      h_i = h_i->next();
      if (to_del != e->halfedge()) erase(to_del);
    } while (h_i != e->halfedge());
  }
  /* Check procedure for bottom face. */
  if (f_t_he.size() == 3) {
    /* Due to bottom nature, we must preserve side of v. */
    HalfedgeRef s_h_t = e->halfedge()->twin()->next()->twin(); /*second inner half-edge twin. */
    HalfedgeRef l_h_t = e->halfedge()->twin()->next()->next()->twin(); /*last inner half-edge twin. */
    EdgeRef l_e = l_h_t->edge();

    /*Adjust half-egdes prior to erasing. */
    s_h_t->set_neighbors(s_h_t->next(), l_h_t, s_h_t->vertex(), s_h_t->edge(), s_h_t->face());
    s_h_t->vertex()->_halfedge = s_h_t;
    s_h_t->edge()->_halfedge = s_h_t;
    l_h_t->set_neighbors(l_h_t->next(), s_h_t, l_h_t->vertex(), s_h_t->edge(), l_h_t->face());
    l_h_t->vertex()->_halfedge = l_h_t;
    l_h_t->edge()->_halfedge = l_h_t;

    /*Delete second edge*/
    erase(l_e);
    /*Delete face*/
    erase(f_t);
    /*Delete inner half edges except main. */
    h_i = e->halfedge()->twin(); /*first inner half-edge. */
    do {
      HalfedgeRef to_del = h_i;
      h_i = h_i->next();
      if (to_del != e->halfedge()) erase(to_del);
    } while (h_i != e->halfedge()->twin());
  }

  /* Adjust values prior to erasing. */
  if (f_he.size() != 3) 
    h_c->set_neighbors(h_c->next()->next(), h_c->twin(), h_c->vertex(), h_c->edge(), h_c->face());
  if (f_t_he.size() != 3)
    h_t_c->set_neighbors(h_t_c->next()->next(), h_t_c->twin(), h_t_c->vertex(), h_t_c->edge(), h_t_c->face());
  f->_halfedge = h_c;
  f_t->_halfedge = h_t_c;

  erase(e->halfedge());
  erase(e->halfedge()->twin());
  erase(e);
  erase(v_t);

  return v;
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

    /* Define containers for all necessary components. */
    vector<FaceRef> f_lst;
    vector<VertexRef> vn_lst;
    vector<HalfedgeRef> hn_lst;
    vector<vector<HalfedgeRef>> h_lst;
    vector<vector<EdgeRef>> e_lst;

    for (unsigned int i = 0; i < f->degree(); i++) {
      /* Create side_face. f_s: face_side. */
      FaceRef f_s = new_face();
      /* Collect new components for side face in container. */
      vector<HalfedgeRef> hi_lst;
      vector<EdgeRef> ei_lst;
      /* Iterate side-face sides, collect items for later arranging. */
      for (int j = 0; j < 3; j++) {
        hi_lst.push_back(new_halfedge());
        if (j < 2) ei_lst.push_back(new_edge()); /* 0: face_edge, 1: face_left_side_edge. */
      }
      f_lst.push_back(f_s);
      vn_lst.push_back(new_vertex());
      hn_lst.push_back(new_halfedge());
      h_lst.push_back(hi_lst);
      e_lst.push_back(ei_lst);
    }

    /* Create new face to return. */
    FaceRef f_new = new_face();

    HalfedgeRef f_i = f->halfedge();
    /* Iterate over face half-edges, which represent all side faces. */
    for (unsigned int i = 0; i < f->degree(); i++) {
      unsigned int i_prev = (i == 0) ? f->degree() - 1 : i - 1;
      unsigned int i_next = (i == f->degree() - 1) ? 0 : i + 1;

      /* Save values for later retrieval. */
      HalfedgeRef f_ip = f_i->next();

      /* Adjust new face half_edge and neighbors. */
      hn_lst[i]->set_neighbors(hn_lst[i_next], h_lst[i][0], vn_lst[i], e_lst[i][0], f_new);
      // vn_lst[i]->pos = Vec3(f_i->vertex()->pos.x, f_i->vertex()->pos.y * 2, f_i->vertex()->pos.z); // DEBUG SIZE
      vn_lst[i]->pos = f_i->vertex()->pos;
      f_new->_halfedge = hn_lst[i];

      /* Associate side face with first half_edge. */
      f_lst[i]->_halfedge = h_lst[i][0];

      /* Adjust side face half_edges. */

      /* Top half-edge */
      h_lst[i][0]->set_neighbors(h_lst[i][1], hn_lst[i], vn_lst[i_next], e_lst[i][0], f_lst[i]);
      vn_lst[i_next]->_halfedge = h_lst[i][0];
      e_lst[i][0]->_halfedge = h_lst[i][0];

      /* Left half-edge */
      h_lst[i][1]->set_neighbors(f_i, h_lst[i_prev][2], vn_lst[i], e_lst[i][1], f_lst[i]);
      vn_lst[i]->_halfedge = h_lst[i][1];
      e_lst[i][1]->_halfedge = h_lst[i][1];

      /* Bottom half-edge */
      f_i->set_neighbors(h_lst[i][2], f_i->twin(), f_i->vertex(), f_i->edge(), f_lst[i]);

      /* Right half-edge */
      h_lst[i][2]->set_neighbors(h_lst[i][0], h_lst[i_next][1], f_ip->vertex(), e_lst[i_next][1], f_lst[i]);
      f_ip->vertex()->_halfedge = h_lst[i][2];
      e_lst[i_next][1]->_halfedge = h_lst[i][2];

      f_i = f_ip;
    }
    erase(f);
    return f_new;
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


    for (size_t i = 0; i < new_halfedges.size(); i++) {
      Vec3 tangent_vec = tangent_offset * face->center().unit();
      new_halfedges[i]->vertex()->pos = start_positions[i] + tangent_vec;
      Vec3 normal_vec = - normal_offset * face->normal().unit();
      new_halfedges[i]->vertex()->pos = start_positions[i] + normal_vec;
    }
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
        validate();
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
    VertexRef start_v = vertices_begin();
    VertexRef end_v = vertices_end();
    for (VertexRef v = start_v; v != end_v; v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    EdgeRef start_e = edges_begin();
    EdgeRef end_e = edges_end();
    for (EdgeRef e = start_e; e != end_e; e++) {
        HalfedgeRef first_he = e->halfedge();
        HalfedgeRef second_he = first_he->twin();

        /* Get both vertices to later get positions. */
        VertexRef first_v = first_he->vertex();
        VertexRef second_v = second_he->vertex();

        Vec3 avg = (first_v->pos + second_v->pos) / 2;

        e->new_pos = avg;
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    FaceRef start_f = faces_begin();
    FaceRef end_f = faces_end();
    for (FaceRef f = start_f; f != end_f; f++) {
        f->new_pos = f->center();
    }
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
    FaceRef start_face = faces_begin();
    FaceRef end_face = faces_end();
    for (FaceRef f = start_face; f != end_face; f++) {
        /* Similar to linear, just set them to their center. */
        f->new_pos = f->center();
    }

    // Edges
    EdgeRef start_edge = edges_begin();
    EdgeRef end_edge = edges_end();
    for (EdgeRef e = start_edge; e != end_edge; e++) {
        /* Grab edges and faces. */
        HalfedgeRef he = e->halfedge();
        HalfedgeRef he_twin = he->twin();
        FaceRef f = he->face();
        FaceRef f_twin = he_twin->face();

        /* Grab specific positions of edges and faces. */
        Vec3 he_pos = he->vertex()->pos;
        Vec3 he_twin_pos = he_twin->vertex()->pos;
        /* Use centers since those are our updated faces' positions. */
        Vec3 f_pos = f->center();
        Vec3 f_twin_pos = f_twin->center();


        /* Calculate average and set value. */
        Vec3 avg_pos = (he_pos + he_twin_pos + f_pos + f_twin_pos) / 4;
        e->new_pos = avg_pos;
    }

    // Vertices
    VertexRef start_v = vertices_begin();
    VertexRef end_v = vertices_end();
    for (VertexRef v = start_v; v != end_v; v++) {
        /* # of faces / edges. */
        unsigned int n = v->degree();

        /* Get counters for averages. */
        Vec3 total_face_new_pos;
        Vec3 total_edge_pos;

        /* Set-up first edge. */
        HalfedgeRef start_he = v->halfedge();
        HalfedgeRef curr_he = start_he;

        /* Loop over all edges and get totals for positions. */
        do {
            FaceRef curr_f = curr_he->face();
            Vec3 curr_f_pos = curr_f->center();
            total_face_new_pos += curr_f_pos;

            Vec3 curr_e_center = curr_he->edge()->center();
            total_edge_pos += curr_e_center;

            curr_he = curr_he->twin()->next();
        } while (curr_he != start_he);

        /* Q is the average of all new face position for faces containing v. */
        Vec3 Q = total_face_new_pos /= n;
        /* R is the average of all original edge midpoints for edges containing v. */
        Vec3 R = total_edge_pos /= n;
        /* S is the original vertex position for vertex v. */
        Vec3 S = v->pos;

        /* New position is an average of averages. */
        Vec3 v_new_pos = (Q + (2 * R) + ((n - 3) * S)) / n;
        v->new_pos = v_new_pos;
    }
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

        /* Double check collapse_edge*/ /* TODO: BUG HERE! */
        /**
         * Reasons: 1) key may not exist; never added to map before
         * 2) way we are creating the key; maybe one vertex may/may-not exist
        */
        Mat4 e_quadric = vertex_quadrics[e->halfedge()->vertex()] +
          vertex_quadrics[e->halfedge()->twin()->vertex()];

        /* Copy of quadric - set last column and last row and col and update*/
        Mat4 A = Mat4(
                  Vec4(e_quadric[0].xyz(), 0), 
                  Vec4(e_quadric[1].xyz(), 0), 
                  Vec4(e_quadric[2].xyz(), 0), 
                  Vec4(Vec3(), 1));
        
        Vec3 n_optimal;
        float n_cost;
        if (A.det() == 0) { /* Non-invertible if det is zero. */
          /* If inverse does not exist, cost function is minimized outside
          vertices of edge - point along edge beyond two vertices.
          i.e. our optimum point must lie at one of the vertices. TODO: Compute
          cost for both vertices, and choose the one with lower cost. */
          float cost_one = dot(Vec4(e->halfedge()->vertex()->pos, 1), 
            e_quadric * Vec4(e->halfedge()->vertex()->pos, 1));
          float cost_two = dot(Vec4(e->halfedge()->twin()->vertex()->pos, 1), 
            e_quadric * Vec4(e->halfedge()->twin()->vertex()->pos, 1));
          n_optimal = (cost_one < cost_two) ? 
            e->halfedge()->vertex()->pos : 
            e->halfedge()->twin()->vertex()->pos;
          n_cost = (cost_one < cost_two) ? cost_one : cost_two;
        } else {
          Vec3 b = -e_quadric[3].xyz();
          n_optimal = A.inverse() * b;
          n_cost = dot(Vec4(n_optimal, 1), e_quadric * Vec4(n_optimal, 1)); /* x^T Kx*/
        }

        /*Store information. */
        this->optimal = n_optimal;
        this->cost = n_cost;
        this->edge = e;
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

    /* Loop through all faces in the shape. Build face_quadrics. */
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
      /* Face quadric: outer(V4(normal.x,y,z, d = negative dot (N = face normal, p = any point of face)), V4)*/
      Vec4 f_vec = Vec4(f->normal(), dot(f->normal(), f->center()));
      Mat4 f_quadric = outer(f_vec, f_vec);
      face_quadrics[f] = f_quadric;
    }

    /* Loop through all vertices in shape. Build vertex_quadrics. */
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
      /* Loop through current v's halfedges, and therefore faces. */
      HalfedgeRef h_i = v->halfedge();
      /* Start vertex quadric sum. */
      Mat4 v_quadric = Mat4::Zero;
      do {
        /* Retrieve face, add sum. */
        v_quadric += face_quadrics[h_i->face()];
        h_i = h_i->twin()->next();
      } while (h_i != v->halfedge());
      /* Vertex quadric sum computed. Add to map. */
      vertex_quadrics[v] = v_quadric;
    }

    /* Loop through all edges in shape. Build edge_queue and edge_records. */
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
      Edge_Record e_r(vertex_quadrics, e);
      edge_records[e] = e_r;
      edge_queue.insert(e_r);
    }

    /* Best edge can collapse only if intersection of vertex neighbors is exactly two*/
    Edge_Record e_check = edge_queue.top();
    VertexRef v_check = e_check.edge->halfedge()->vertex(); /*Side one*/
    VertexRef v_t_check = e_check.edge->halfedge()->twin()->vertex(); /*Side two*/
    set<VertexRef> v_set = {v_check};
    set<VertexRef> v_t_set = {v_t_check};
    set<VertexRef> v_intersect;
    HalfedgeRef hv_i = v_check->halfedge();
    do {
      v_set.insert(hv_i->twin()->vertex());
      hv_i = hv_i->twin()->next();
    } while (hv_i != v_check->halfedge());

    hv_i = v_t_check->halfedge();
    do {
      v_t_set.insert(hv_i->twin()->vertex());
      hv_i = hv_i->twin()->next();
    } while (hv_i != v_t_check->halfedge());

    set_intersection(v_set.begin(), v_set.end(), v_t_set.begin(), v_t_set.end(),
      inserter(v_intersect, v_intersect.begin()));
    
    if (n_vertices() <= 3 && v_intersect.size() != 2) return false;

    /* Collapse edges until size_target is reached. */
    size_t size_target = edge_queue.size() / 4;
    while (edge_queue.size() > size_target) {
      /* Minimum-priority queue. Method top retrieves edge with smallest cost. */
      Edge_Record e_best = edge_queue.top();
      /* Compute combined quadrics for new vertex. */
      VertexRef v = e_best.edge->halfedge()->vertex(); /*Side one*/
      VertexRef v_t = e_best.edge->halfedge()->twin()->vertex(); /*Side two*/

      Mat4 vn_quadric = vertex_quadrics[v] + vertex_quadrics[v_t];
      
      /* Collect all edges connected to the edge-to-collapse. */
      vector<EdgeRef> e_connected;
      HalfedgeRef hv = v->halfedge();
      HalfedgeRef hv_t = v_t->halfedge();
      do {
        /* Start with side one*/
        if (hv->edge() != e_best.edge) e_connected.push_back(hv->edge());
        hv = hv->twin()->next();
      } while (hv != v->halfedge());
      do {
        /* Continue with side two*/
        if (hv_t->edge() != e_best.edge) e_connected.push_back(hv_t->edge());
        hv_t = hv_t->twin()->next();
      } while (hv_t != v_t->halfedge());

      /* Remove edges from priority queue. */
      for (EdgeRef e_curr : e_connected) {
        edge_queue.remove(edge_records[e_curr]); /* ABORT OCCURS HERE*/
      }
      
      /* Collapse edges. */
      auto collapse_result = collapse_edge(e_best.edge);

      /* Remove all erased edges from priority queue. */
      for (EdgeRef e_erased : eerased) {
        edge_queue.remove(edge_records[e_erased]);
        if (find(e_connected.begin(), e_connected.end(), e_erased) != e_connected.end())
          e_connected.erase(find(e_connected.begin(), e_connected.end(), e_erased));
      }
      do_erase();

      /* No value for collapsed edge. Can't simplify. Return false. */
      if (!collapse_result.has_value()) return false;

      VertexRef v_collapsed = collapse_result.value(); /* Potential ERROR HERE.*/

      /* Update vertex quadric. */
      vertex_quadrics[v_collapsed] = vn_quadric;

      /* Add edges to priority queue post_collapse. */
      for (EdgeRef e_curr : e_connected) { /* BUG HERE */
        Edge_Record new_record = Edge_Record(vertex_quadrics, e_curr);
        edge_records[e_curr] = new_record;
        edge_queue.insert(new_record);
      }

      edge_queue.pop();
    }
    
    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return true;
}
