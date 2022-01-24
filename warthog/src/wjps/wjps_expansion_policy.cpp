#include "wjps_expansion_policy.h"
#include "jps.h"

warthog::wjps_expansion_policy::wjps_expansion_policy(vl_gridmap& map)
    : expansion_policy(map.height()*map.width()),
    map_(map), local_map_(3, 3), expander_(&local_map_)
{
    for (int j = 0; j < 8; j++) {
        tables_[j] = new uint8_t[NBHOOD_TABLE_SIZE];
        for (size_t i = 0; i < NBHOOD_TABLE_SIZE; i++) {
            tables_[j][i] = warthog::jps::ALL;
        }
    }

    extra_ = new wjps_extra[map.width() * map.height()];
    int next_cost = 1;
    costs_[0] = 0;
    for (uint32_t id = 0; id < map.width()*map.height(); id++) {
        auto cost = map.get_label(id);
        bool found = false;
        for (int i = 0; i < next_cost; i++) {
            if (cost == costs_[i]) {
                map.set_label(id, i);
                found = true;
                break;
            }
        }
        if (!found) {
            if (next_cost == 3) {
                throw std::logic_error("too many different-cost tiles on this map");
            }
            costs_[next_cost] = cost;
            map.set_label(id, next_cost);
            next_cost++;
        }
    }

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

warthog::wjps_expansion_policy::~wjps_expansion_policy()
{
    delete[] extra_;
    for (int j = 0; j < 8; j++) {
        delete[] tables_[j];
    }
}

warthog::search_node*
warthog::wjps_expansion_policy::generate_start_node(warthog::problem_instance* pi)
{
    uint32_t max_id = map_.header_width() * map_.header_height();
    if ((uint32_t) pi->start_id_ >= max_id) {
        return 0;
    }
    uint32_t padded_id = map_.to_padded_id((uint32_t) pi->start_id_);
    if (map_.get_label(padded_id) == 0) {
        return 0;
    }

    nbhood_labels nb = nbhood(padded_id);

    // Find initial successor set
    auto& extra = get_extra(padded_id, pi);
    if (map_.get_label(nb.n)) {
        extra.successors_ |= warthog::jps::NORTH;
        prospect(padded_id, nb.n, warthog::jps::NORTH, vertical_cost(nb.n), pi);
        if (map_.get_label(nb.e) && map_.get_label(nb.ne)) {
            extra.successors_ |= warthog::jps::NORTHEAST;
            prospect(padded_id, nb.ne, warthog::jps::NORTHEAST, diagonal_cost(nb.n), pi);
        }
        if (map_.get_label(nb.w) && map_.get_label(nb.nw)) {
            extra.successors_ |= warthog::jps::NORTHWEST;
            prospect(padded_id, nb.nw, warthog::jps::NORTHWEST, diagonal_cost(nb.nw), pi);
        }
    }
    if (map_.get_label(nb.s)) {
        extra.successors_ |= warthog::jps::SOUTH;
        prospect(padded_id, nb.s, warthog::jps::SOUTH, vertical_cost(nb.h), pi);
        if (map_.get_label(nb.e) && map_.get_label(nb.se)) {
            extra.successors_ |= warthog::jps::SOUTHEAST;
            prospect(padded_id, nb.se, warthog::jps::SOUTHEAST, diagonal_cost(nb.h), pi);
        }
        if (map_.get_label(nb.w) && map_.get_label(nb.sw)) {
            extra.successors_ |= warthog::jps::SOUTHWEST;
            prospect(padded_id, nb.sw, warthog::jps::SOUTHWEST, diagonal_cost(nb.w), pi);
        }
    }
    if (map_.get_label(nb.e)) {
        extra.successors_ |= warthog::jps::EAST;
        prospect(padded_id, nb.e, warthog::jps::EAST, horizontal_cost(nb.h), pi);
    }
    if (map_.get_label(nb.w)) {
        extra.successors_ |= warthog::jps::WEST;
        prospect(padded_id, nb.e, warthog::jps::WEST, horizontal_cost(nb.w), pi);
    }

    return generate(padded_id);
}

warthog::search_node*
warthog::wjps_expansion_policy::generate_target_node(warthog::problem_instance* pi)
{
    uint32_t max_id = map_.header_width() * map_.header_height();
    if ((uint32_t) pi->target_id_ >= max_id) {
        return 0;
    }
    uint32_t padded_id = map_.to_padded_id((uint32_t) pi->target_id_);
    if (map_.get_label(padded_id) == 0) {
        return 0;
    }
    return generate(padded_id);
}

void
warthog::wjps_expansion_policy::expand(warthog::search_node* node, warthog::problem_instance* pi)
{
    reset();

    uint32_t id = node->get_id();
    auto nb = nbhood(id);

    // for each direction
    if (extra_[id].successors_ & warthog::jps::NORTHWEST) {
        double cost = diagonal_cost(nb.nw);
        add_neighbour(generate(nb.nw), cost);
        reach(id, nb.nw, warthog::jps::NORTHWEST, node->get_g() + cost, pi);
    }
    if (extra_[id].successors_ & warthog::jps::NORTH) {
        double cost = vertical_cost(nb.n);
        add_neighbour(generate(nb.n), cost);
        reach(id, nb.n, warthog::jps::NORTH, node->get_g() + cost, pi);
    }
    if (extra_[id].successors_ & warthog::jps::NORTHEAST) {
        double cost = diagonal_cost(nb.n);
        add_neighbour(generate(nb.ne), cost);
        reach(id, nb.ne, warthog::jps::NORTHEAST, node->get_g() + cost, pi);
    }

    if (extra_[id].successors_ & warthog::jps::WEST) {
        double cost = horizontal_cost(nb.w);
        add_neighbour(generate(nb.w), cost);
        reach(id, nb.w, warthog::jps::WEST, node->get_g() + cost, pi);
    }
    if (extra_[id].successors_ & warthog::jps::EAST) {
        double cost = horizontal_cost(nb.h);
        add_neighbour(generate(nb.e), cost);
        reach(id, nb.e, warthog::jps::EAST, node->get_g() + cost, pi);
    }

    if (extra_[id].successors_ & warthog::jps::SOUTHWEST) {
        double cost = diagonal_cost(nb.w);
        add_neighbour(generate(nb.sw), cost);
        reach(id, nb.sw, warthog::jps::SOUTHWEST, node->get_g() + cost, pi);
    }
    if (extra_[id].successors_ & warthog::jps::SOUTH) {
        double cost = vertical_cost(nb.h);
        add_neighbour(generate(nb.n), cost);
        reach(id, nb.s, warthog::jps::SOUTH, node->get_g() + cost, pi);
    }
    if (extra_[id].successors_ & warthog::jps::SOUTHEAST) {
        double cost = diagonal_cost(nb.h);
        add_neighbour(generate(nb.se), cost);
        reach(id, nb.se, warthog::jps::SOUTHEAST, node->get_g() + cost, pi);
    }
}

void warthog::wjps_expansion_policy::reach(
        uint32_t from, uint32_t to,
        warthog::jps::direction direction, double g,
        warthog::problem_instance* pi)
{
    wjps_extra& extra = get_extra(to, pi);
    auto nb = nbhood(to);
    warthog::search_node* from_node = generate(from);
    warthog::search_node* to_node = generate(to);
    if (from_node->get_search_number() != to_node->get_search_number() || g < to_node->get_g()) {
        // determine successor set and prospect to prune
        extra.prospective_g_ = g;
        extra.prospective_parent_ = from;
        extra.moving_direction_ = direction;

        extra.successors_ = nbhood_successors(to, direction);

        // for each direction
        if (extra.successors_ & warthog::jps::NORTHWEST) {
            prospect(to, nb.nw, warthog::jps::NORTHWEST, g + diagonal_cost(nb.nw), pi);
        }
        if (extra.successors_ & warthog::jps::NORTH) {
            prospect(to, nb.n, warthog::jps::NORTH, g + vertical_cost(nb.n), pi);
        }
        if (extra.successors_ & warthog::jps::NORTHEAST) {
            prospect(to, nb.ne, warthog::jps::NORTHEAST, g + diagonal_cost(nb.n), pi);
        }

        if (extra.successors_ & warthog::jps::WEST) {
            prospect(to, nb.w, warthog::jps::WEST, g + horizontal_cost(nb.w), pi);
        }
        if (extra.successors_ & warthog::jps::EAST) {
            prospect(to, nb.e, warthog::jps::EAST, g + horizontal_cost(nb.h), pi);
        }

        if (extra.successors_ & warthog::jps::SOUTHWEST) {
            prospect(to, nb.sw, warthog::jps::SOUTHWEST, g + diagonal_cost(nb.w), pi);
        }
        if (extra.successors_ & warthog::jps::SOUTH) {
            prospect(to, nb.s, warthog::jps::SOUTH, g + vertical_cost(nb.h), pi);
        }
        if (extra.successors_ & warthog::jps::SOUTHEAST) {
            prospect(to, nb.se, warthog::jps::SOUTHEAST, g + diagonal_cost(nb.h), pi);
        }
    } else if (
        from_node->get_search_number() == to_node->get_search_number() && g == to_node->get_g()
    ) {
        // intersect successor set
    }
}

void warthog::wjps_expansion_policy::prospect(
        uint32_t from, uint32_t to,
        warthog::jps::direction direction, double g,
        warthog::problem_instance* pi)
{
    constexpr int ORTHO_DIRS =
        warthog::jps::NORTH | warthog::jps::EAST | warthog::jps::SOUTH | warthog::jps::WEST;

    wjps_extra& extra = get_extra(to, pi);
    bool is_preferred_dir = (direction & ORTHO_DIRS) && (extra.moving_direction_ & ~ORTHO_DIRS);
    if (g < extra.prospective_g_ || is_preferred_dir) {
        // remove successor from other
        extra_[extra.prospective_parent_].successors_ &= ~extra.moving_direction_;
        // establish new parent
        extra.prospective_g_ = g;
        extra.prospective_parent_ = from;
        extra.moving_direction_ = direction;
    } else {
        // from node does not have to node as a successor
        extra_[from].successors_ &= ~direction;
    }
}

int warthog::wjps_expansion_policy::nbhood_successors(
        uint32_t to, warthog::jps::direction going)
{
    nbhood_labels nb = nbhood(to);
    uint8_t* table;
    uint32_t source;
    switch (going) {
        case warthog::jps::NORTHWEST:
            table = tables_[0];
            source = 2 + 2 * 3;
            break;
        case warthog::jps::NORTH:
            table = tables_[1];
            source = 1 + 2 * 3;
            break;
        case warthog::jps::NORTHEAST:
            table = tables_[2];
            source = 0 + 2 * 3;
            break;
        case warthog::jps::WEST:
            table = tables_[3];
            source = 2 + 1 * 3;
            break;
        case warthog::jps::EAST:
            table = tables_[4];
            source = 0 + 1 * 3;
            break;
        case warthog::jps::SOUTHWEST:
            table = tables_[5];
            source = 2 + 0 * 3;
            break;
        case warthog::jps::SOUTH:
            table = tables_[6];
            source = 1 + 0 * 3;
            break;
        case warthog::jps::SOUTHEAST:
            table = tables_[7];
            source = 0 + 0 * 3;
            break;
        default:
            assert(false);
            return 0;
    }
    size_t index = map_.get_label(nb.nw)
            + map_.get_label(nb.n)  * 3
            + map_.get_label(nb.ne) * 3 * 3
            + map_.get_label(nb.w)  * 3 * 3 * 3
            + map_.get_label(nb.h)  * 3 * 3 * 3 * 3
            + map_.get_label(nb.e)  * 3 * 3 * 3 * 3 * 3
            + map_.get_label(nb.sw) * 3 * 3 * 3 * 3 * 3 * 3
            + map_.get_label(nb.s)  * 3 * 3 * 3 * 3 * 3 * 3 * 3
            + map_.get_label(nb.se) * 3 * 3 * 3 * 3 * 3 * 3 * 3 * 3;
    if (table[index] == warthog::jps::ALL) {
        local_map_.set_label(local_nb_.nw, costs_[map_.get_label(nb.nw)]);
        local_map_.set_label(local_nb_.n,  costs_[map_.get_label(nb.n)]);
        local_map_.set_label(local_nb_.ne, costs_[map_.get_label(nb.ne)]);
        local_map_.set_label(local_nb_.w,  costs_[map_.get_label(nb.w)]);
        local_map_.set_label(local_nb_.h,  costs_[map_.get_label(nb.h)]);
        local_map_.set_label(local_nb_.e,  costs_[map_.get_label(nb.e)]);
        local_map_.set_label(local_nb_.sw, costs_[map_.get_label(nb.sw)]);
        local_map_.set_label(local_nb_.s,  costs_[map_.get_label(nb.s)]);
        local_map_.set_label(local_nb_.se, costs_[map_.get_label(nb.se)]);
        table[index] = calculate_successors(source);
    }
    return table[index];
}

int warthog::wjps_expansion_policy::calculate_successors(uint32_t source)
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
                if (new_is_ortho && !existing_is_ortho) {
                    // tiebreak in favor of ortho-last
                    n->set_parent(current->get_id());
                }
            }
        }
    }

    int successors = 0;
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
    return successors;
}
