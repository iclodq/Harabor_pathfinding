#include "nbcache.h"

warthog::nbcache::nbcache() : local_map_(3, 3), expander_(&local_map_), pqueue_()
{
    local_nb_.nw = local_map_.to_padded_id(0, 0);
    local_nb_.n = local_map_.to_padded_id(1, 0);
    local_nb_.ne = local_map_.to_padded_id(2, 0);
    local_nb_.w = local_map_.to_padded_id(0, 1);
    local_nb_.h = local_map_.to_padded_id(1, 1);
    local_nb_.e = local_map_.to_padded_id(2, 1);
    local_nb_.sw = local_map_.to_padded_id(0, 2);
    local_nb_.s = local_map_.to_padded_id(1, 2);
    local_nb_.se = local_map_.to_padded_id(2, 2);
}

warthog::jps::direction warthog::nbcache::successors(
        vl_gridmap& map, nbhood_labels& nb, warthog::jps::direction going)
{
    local_map_.set_label(local_nb_.nw, map.get_label(nb.nw));
    local_map_.set_label(local_nb_.n,  map.get_label(nb.n));
    local_map_.set_label(local_nb_.ne, map.get_label(nb.ne));
    local_map_.set_label(local_nb_.w,  map.get_label(nb.w));
    local_map_.set_label(local_nb_.h,  map.get_label(nb.h));
    local_map_.set_label(local_nb_.e,  map.get_label(nb.e));
    local_map_.set_label(local_nb_.sw, map.get_label(nb.sw));
    local_map_.set_label(local_nb_.s,  map.get_label(nb.s));
    local_map_.set_label(local_nb_.se, map.get_label(nb.se));
    uint32_t source;
    switch (going) {
        case warthog::jps::NORTHWEST:
            source = 2 + 2 * 3;
            break;
        case warthog::jps::NORTH:
            source = 1 + 2 * 3;
            break;
        case warthog::jps::NORTHEAST:
            source = 0 + 2 * 3;
            break;
        case warthog::jps::WEST:
            source = 2 + 1 * 3;
            break;
        case warthog::jps::EAST:
            source = 0 + 1 * 3;
            break;
        case warthog::jps::SOUTHWEST:
            source = 2 + 0 * 3;
            break;
        case warthog::jps::SOUTH:
            source = 1 + 0 * 3;
            break;
        case warthog::jps::SOUTHEAST:
            source = 0 + 0 * 3;
            break;
        default:
            assert(false);
            return warthog::jps::NONE;
    }

    return calculate_successors(source);
}

warthog::jps::direction warthog::nbcache::calculate_successors(uint32_t source)
{
    warthog::problem_instance pi(source);

    warthog::search_node* start = expander_.generate_start_node(&pi);
    pi.start_id_ = start->get_id();
    start->init(pi.instance_id_, warthog::SN_ID_MAX, 0, 0);
    pqueue_.push(start);

    while (pqueue_.size()) {
        warthog::search_node* current = pqueue_.pop();
        current->set_expanded(true);
        expander_.expand(current, &pi);

        warthog::search_node* n;
        warthog::cost_t cost_to_n;

        for (expander_.first(n, cost_to_n); n != 0; expander_.next(n, cost_to_n)) {
            warthog::cost_t gval = current->get_g() + cost_to_n;
            if (n->get_search_number() != current->get_search_number()) {
                n->init(current->get_search_number(), current->get_id(), gval, gval);
                pqueue_.push(n);
            } else if (gval < n->get_g()) {
                n->relax(gval, current->get_id());
                pqueue_.decrease_key(n);
            } else if (gval == n->get_g()) {
                uint32_t h = n->get_id();
                uint32_t p = n->get_parent();
                bool existing_is_ortho =
                    p == h - 1 || p == h - local_map_.width() ||
                    p == h + 1 || p == h + local_map_.width();
                uint32_t c = current->get_id();
                bool new_is_ortho =
                    c == h - 1 || c == h - local_map_.width() ||
                    c == h + 1 || c == h + local_map_.width();
                if (
                    (new_is_ortho && !existing_is_ortho) ||
                    (new_is_ortho == existing_is_ortho && c == local_nb_.h)
                ) {
                    // tiebreak in favor of ortho-last
                    n->set_parent(current->get_id());
                }
            }
        }
    }

    int successors = warthog::jps::NONE;
    warthog::search_node* n = expander_.generate(local_nb_.nw);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::NORTHWEST;
    }
    n = expander_.generate(local_nb_.n);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::NORTH;
    }
    n = expander_.generate(local_nb_.ne);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::NORTHEAST;
    }
    n = expander_.generate(local_nb_.w);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::WEST;
    }
    n = expander_.generate(local_nb_.e);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::EAST;
    }
    n = expander_.generate(local_nb_.sw);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::SOUTHWEST;
    }
    n = expander_.generate(local_nb_.s);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::SOUTH;
    }
    n = expander_.generate(local_nb_.se);
    if (n->get_search_number() == pi.instance_id_ && n->get_parent() == local_nb_.h) {
        successors |= warthog::jps::SOUTHEAST;
    }
    return (warthog::jps::direction) successors;
}
