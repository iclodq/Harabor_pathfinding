#include "wjps_expansion_policy.h"
#include "jps.h"

struct jumpcache
{
    uint32_t version_;
    uint32_t target_;
    warthog::cost_t g_;
};

struct warthog::wjps_extra
{
    warthog::cost_t prospective_g_;
    uint32_t search_number_;
    warthog::jps::direction moving_direction_;
    uint32_t prospective_parent_;
    uint8_t successors_;

    uint8_t successor_sets_[8];
    jumpcache jump_cache_[4];
};

warthog::wjps_expansion_policy::wjps_expansion_policy(
        nbcache& nbcache, vl_gridmap& map, cost_table& costs)
    : expansion_policy(map.height()*map.width()),
    map_(map), costs_(costs), nbcache_(nbcache)
{
    extra_ = new wjps_extra[map.width() * map.height()];
    for (uint32_t id = 0; id < map.width()*map.height(); id++) {
        for (int i = 0; i < 8; i++) {
            extra_[id].successor_sets_[i] = warthog::jps::ALL;
        }
        for (int i = 0; i < 4; i++) {
            extra_[id].jump_cache_[i].version_ = 0;
        }
    }

    row_versions_ = new uint32_t[map.height()];
    for (uint32_t i = 0; i < map.height(); i++) {
        row_versions_[i] = 1;
    }
    col_versions_ = new uint32_t[map.width()];
    for (uint32_t i = 0; i < map.width(); i++) {
        col_versions_[i] = 1;
    }
}

warthog::wjps_expansion_policy::~wjps_expansion_policy()
{
    delete[] extra_;
    delete[] row_versions_;
    delete[] col_versions_;
}

size_t warthog::wjps_expansion_policy::mem()
{
    return expansion_policy::mem() + sizeof(*this) + map_.mem() + nbcache_.mem()
            + sizeof(wjps_extra) * map_.width() * map_.height()
            + sizeof(uint32_t) * (map_.width() + map_.height());
}

warthog::wjps_extra&
warthog::wjps_expansion_policy::get_extra(uint32_t id, warthog::problem_instance* pi) {
    wjps_extra& extra = extra_[id];
    if (extra.search_number_ != pi->instance_id_) {
        extra.search_number_ = pi->instance_id_;
        extra.prospective_g_ = warthog::COST_MAX;
        extra.prospective_parent_ = 0;
        extra.successors_ = 0;
        extra.moving_direction_ = warthog::jps::NONE;
    }
    return extra;
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
        prospect(padded_id, nb.w, warthog::jps::WEST, horizontal_cost(nb.w), pi);
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
        jump_nw(nb, node->get_g(), extra_[id].successors_, pi);
    }
    if (extra_[id].successors_ & warthog::jps::NORTH) {
        jump_north(id, nb, node->get_g(), 0.0, pi);
    }
    if (extra_[id].successors_ & warthog::jps::NORTHEAST) {
        jump_ne(nb, node->get_g(), extra_[id].successors_, pi);
    }

    if (extra_[id].successors_ & warthog::jps::WEST) {
        jump_west(id, nb, node->get_g(), 0.0, pi);
    }
    if (extra_[id].successors_ & warthog::jps::EAST) {
        jump_east(id, nb, node->get_g(), 0.0, pi);
    }

    if (extra_[id].successors_ & warthog::jps::SOUTHWEST) {
        jump_sw(nb, node->get_g(), extra_[id].successors_, pi);
    }
    if (extra_[id].successors_ & warthog::jps::SOUTH) {
        jump_south(id, nb, node->get_g(), 0.0, pi);
    }
    if (extra_[id].successors_ & warthog::jps::SOUTHEAST) {
        jump_se(nb, node->get_g(), extra_[id].successors_, pi);
    }
}

template<int SLOT, typename F>
void warthog::wjps_expansion_policy::calculate_jump(
    uint32_t start, int delta, uint32_t version, F cost)
{
    auto jump_dist = 0;
    uint32_t last_id = 0;
    auto jump_cost = 0.0;
    auto nb = nbhood(start);
    do {
        jump_dist += 1;
        nb = nbhood(nb.h + delta);
        last_id = nb.h;
        if (!locally_uniform(nb)) {
            break;
        }
        auto& jc = extra_[nb.h].jump_cache_[SLOT];
        if (jc.version_ == version) {
            jump_cost = jc.g_;
            nb = nbhood(jc.target_);
            break;
        }
    } while (true);

    for (int i = 0; i < jump_dist; i++) {
        last_id -= delta;
        jump_cost += cost(last_id);
        extra_[last_id].jump_cache_[SLOT].target_ = nb.h;
        extra_[last_id].jump_cache_[SLOT].g_ = jump_cost;
        extra_[last_id].jump_cache_[SLOT].version_ = version;
    }

    assert(last_id == start);
}

void warthog::wjps_expansion_policy::jump_west(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi)
{
    auto start = nb.h;
    auto version = row_versions_[start / map_.width()];
    if (extra_[nb.h].jump_cache_[0].version_ != version) {
        calculate_jump<0>(start, -1, version, [this](uint32_t h) {
            return horizontal_cost(h - 1);
        });
    }
    cost += extra_[nb.h].jump_cache_[0].g_;
    nb = nbhood(extra_[nb.h].jump_cache_[0].target_);

    if (start > pi->target_id_ && nb.h < pi->target_id_) {
        // Jump overshoots target, so adjust to target
        nb = nbhood(pi->target_id_);
        cost -= extra_[nb.h].jump_cache_[0].g_;
    }

    if (nb.h == pi->target_id_ || nbhood_successors(nb.h, warthog::jps::WEST) != 0) {
        add_neighbour(generate(nb.h), cost);
        reach(from, nb.h, warthog::jps::WEST, g + cost, pi);
    }
}

void warthog::wjps_expansion_policy::jump_east(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi)
{
    auto start = nb.h;
    auto version = row_versions_[start / map_.width()];
    if (extra_[nb.h].jump_cache_[1].version_ != version) {
        calculate_jump<1>(start, 1, version, [this](uint32_t h) {
            return horizontal_cost(h);
        });
    }
    cost += extra_[nb.h].jump_cache_[1].g_;
    nb = nbhood(extra_[nb.h].jump_cache_[1].target_);

    if (start < pi->target_id_ && nb.h > pi->target_id_) {
        // Jump overshoots target, so adjust to target
        nb = nbhood(pi->target_id_);
        cost -= extra_[nb.h].jump_cache_[1].g_;
    }

    if (nb.h == pi->target_id_ || nbhood_successors(nb.h, warthog::jps::EAST) != 0) {
        add_neighbour(generate(nb.h), cost);
        reach(from, nb.h, warthog::jps::EAST, g + cost, pi);
    }
}

void warthog::wjps_expansion_policy::jump_north(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi)
{
    auto start = nb.h;
    auto version = col_versions_[start % map_.width()];
    if (extra_[nb.h].jump_cache_[2].version_ != version) {
        calculate_jump<2>(start, -(int) map_.width(), version, [this](uint32_t h) {
            return vertical_cost(h - map_.width());
        });
    }
    cost += extra_[nb.h].jump_cache_[2].g_;
    nb = nbhood(extra_[nb.h].jump_cache_[2].target_);

    if (start % map_.width() == pi->target_id_ % map_.width() && start > pi->target_id_ && nb.h < pi->target_id_) {
        // Jump overshoots target, so adjust to target
        nb = nbhood(pi->target_id_);
        cost -= extra_[nb.h].jump_cache_[2].g_;
    }

    if (nb.h == pi->target_id_ || nbhood_successors(nb.h, warthog::jps::NORTH) != 0) {
        add_neighbour(generate(nb.h), cost);
        reach(from, nb.h, warthog::jps::NORTH, g + cost, pi);
    }
}

void warthog::wjps_expansion_policy::jump_south(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi)
{
    auto start = nb.h;
    auto version = col_versions_[start % map_.width()];
    if (extra_[nb.h].jump_cache_[3].version_ != version) {
        calculate_jump<3>(start, map_.width(), version, [this](uint32_t h) {
            return vertical_cost(h);
        });
    }
    cost += extra_[nb.h].jump_cache_[3].g_;
    nb = nbhood(extra_[nb.h].jump_cache_[3].target_);

    if (start % map_.width() == pi->target_id_ % map_.width() && start < pi->target_id_ && nb.h > pi->target_id_) {
        // Jump overshoots target, so adjust to target
        nb = nbhood(pi->target_id_);
        cost -= extra_[nb.h].jump_cache_[3].g_;
    }
    if (nb.h == pi->target_id_ || nbhood_successors(nb.h, warthog::jps::SOUTH) != 0) {
        add_neighbour(generate(nb.h), cost);
        reach(from, nb.h, warthog::jps::SOUTH, g + cost, pi);
    }
}

void warthog::wjps_expansion_policy::jump_nw(
        nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi)
{
    uint32_t from = nb.h;
    double cost = diagonal_cost(nb.nw);
    nb = nbhood(nb.nw);
    while (locally_uniform(nb) && nb.h != pi->target_id_) {
        if (successor_set & warthog::jps::NORTH) {
            jump_north(from, nb, g, cost, pi);
        }
        if (successor_set & warthog::jps::WEST) {
            jump_west(from, nb, g, cost, pi);
        }
        cost += diagonal_cost(nb.nw);
        nb = nbhood(nb.nw);
    }
    add_neighbour(generate(nb.h), cost);
    reach(from, nb.h, warthog::jps::NORTHWEST, g + cost, pi);
}

void warthog::wjps_expansion_policy::jump_ne(
        nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi)
{
    uint32_t from = nb.h;
    double cost = diagonal_cost(nb.n);
    nb = nbhood(nb.ne);
    while (locally_uniform(nb) && nb.h != pi->target_id_) {
        if (successor_set & warthog::jps::NORTH) {
            jump_north(from, nb, g, cost, pi);
        }
        if (successor_set & warthog::jps::EAST) {
            jump_east(from, nb, g, cost, pi);
        }
        cost += diagonal_cost(nb.n);
        nb = nbhood(nb.ne);
    }
    add_neighbour(generate(nb.h), cost);
    reach(from, nb.h, warthog::jps::NORTHEAST, g + cost, pi);
}

void warthog::wjps_expansion_policy::jump_sw(
        nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi)
{
    uint32_t from = nb.h;
    double cost = diagonal_cost(nb.w);
    nb = nbhood(nb.sw);
    while (locally_uniform(nb) && nb.h != pi->target_id_) {
        if (successor_set & warthog::jps::SOUTH) {
            jump_south(from, nb, g, cost, pi);
        }
        if (successor_set & warthog::jps::WEST) {
            jump_west(from, nb, g, cost, pi);
        }
        cost += diagonal_cost(nb.w);
        nb = nbhood(nb.sw);
    }
    add_neighbour(generate(nb.h), cost);
    reach(from, nb.h, warthog::jps::SOUTHWEST, g + cost, pi);
}

void warthog::wjps_expansion_policy::jump_se(
        nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi)
{
    uint32_t from = nb.h;
    double cost = diagonal_cost(nb.h);
    nb = nbhood(nb.se);
    while (locally_uniform(nb) && nb.h != pi->target_id_) {
        if (successor_set & warthog::jps::SOUTH) {
            jump_south(from, nb, g, cost, pi);
        }
        if (successor_set & warthog::jps::EAST) {
            jump_east(from, nb, g, cost, pi);
        }
        cost += diagonal_cost(nb.h);
        nb = nbhood(nb.se);
    }
    add_neighbour(generate(nb.h), cost);
    reach(from, nb.h, warthog::jps::SOUTHEAST, g + cost, pi);
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
        extra.successors_ &= nbhood_successors(to, direction);
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
    if (g < extra.prospective_g_ || (g == extra.prospective_g_ && is_preferred_dir)) {
        // remove successor from other
        if (extra.prospective_parent_ != from) {
            extra_[extra.prospective_parent_].successors_ &= ~extra.moving_direction_;
        }
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
    uint8_t* successor_set;
    switch (going) {
        case warthog::jps::NORTHWEST:
            successor_set = &extra_[nb.h].successor_sets_[0];
            break;
        case warthog::jps::NORTH:
            successor_set = &extra_[nb.h].successor_sets_[1];
            break;
        case warthog::jps::NORTHEAST:
            successor_set = &extra_[nb.h].successor_sets_[2];
            break;
        case warthog::jps::WEST:
            successor_set = &extra_[nb.h].successor_sets_[3];
            break;
        case warthog::jps::EAST:
            successor_set = &extra_[nb.h].successor_sets_[4];
            break;
        case warthog::jps::SOUTHWEST:
            successor_set = &extra_[nb.h].successor_sets_[5];
            break;
        case warthog::jps::SOUTH:
            successor_set = &extra_[nb.h].successor_sets_[6];
            break;
        case warthog::jps::SOUTHEAST:
            successor_set = &extra_[nb.h].successor_sets_[7];
            break;
        default:
            assert(false);
            return 0;
    }
    if (*successor_set == warthog::jps::ALL) {
        *successor_set = nbcache_.successors(map_, nb, going);
    }
    return *successor_set;
}
