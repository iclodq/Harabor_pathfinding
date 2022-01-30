#ifndef WJPS_EXPANSION_POLICY_H
#define WJPS_EXPANSION_POLICY_H

#include "expansion_policy.h"
#include "labelled_gridmap.h"
#include "vl_gridmap_expansion_policy.h"
#include "pqueue.h"

namespace warthog
{

struct wjps_extra
{
    uint32_t search_number_;
    warthog::jps::direction moving_direction_;
    uint8_t successors_;
    uint32_t prospective_parent_;
    warthog::cost_t prospective_g_;
};

struct nbhood_labels
{
    uint32_t nw;
    uint32_t n;
    uint32_t ne;
    uint32_t w;
    uint32_t h;
    uint32_t e;
    uint32_t sw;
    uint32_t s;
    uint32_t se;
};

class wjps_expansion_policy : public expansion_policy
{
public:
    /*
    * This will modify the provided map, changing nonzero labels to 1 for one tile type and 2 for
    * the other (the expansion policy will remember the costs).
    *
    * If there are more than 2 nonzero tile types, an exception is thrown.
    */
    wjps_expansion_policy(vl_gridmap& map);
    ~wjps_expansion_policy();

    virtual void 
    expand(warthog::search_node*, warthog::problem_instance*);

    virtual void
    get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y)
    {
        map_.to_unpadded_xy(nid, (uint32_t&) x, (uint32_t&) y);
    }

    virtual warthog::search_node*
    generate_start_node(warthog::problem_instance* pi);

    virtual warthog::search_node*
    generate_target_node(warthog::problem_instance* pi);

private:
    void reach(
        uint32_t from, uint32_t to,
        warthog::jps::direction direction, double g,
        warthog::problem_instance* pi);

    void prospect(
        uint32_t from, uint32_t to,
        warthog::jps::direction direction, double g,
        warthog::problem_instance* pi);

    int calculate_successors(uint32_t source);
    int nbhood_successors(uint32_t to, warthog::jps::direction going);

    inline wjps_extra& get_extra(uint32_t id, warthog::problem_instance* pi) {
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

    inline double diagonal_cost(uint32_t nw_corner) {
        double sum = costs_[map_.get_label(nw_corner)]
                + costs_[map_.get_label(nw_corner + 1)]
                + costs_[map_.get_label(nw_corner + map_.width())]
                + costs_[map_.get_label(nw_corner + map_.width() + 1)];
        return sum * warthog::DBL_ROOT_TWO / 4.0;
    }

    inline double vertical_cost(uint32_t north) {
        return (costs_[map_.get_label(north)] + costs_[map_.get_label(north + map_.width())]) / 2.0;
    }

    inline double horizontal_cost(uint32_t west) {
        return (costs_[map_.get_label(west)] + costs_[map_.get_label(west + 1)]) / 2.0;
    }

    inline nbhood_labels nbhood(uint32_t around) {
        return {
            around - map_.width() - 1,
            around - map_.width(),
            around - map_.width() + 1,
            around - 1,
            around,
            around + 1,
            around + map_.width() - 1,
            around + map_.width(),
            around + map_.width() + 1,
        };
    }

    inline bool locally_uniform(nbhood_labels nb) {
        return map_.get_label(nb.nw) == map_.get_label(nb.h) &&
                map_.get_label(nb.n) == map_.get_label(nb.h) &&
                map_.get_label(nb.ne) == map_.get_label(nb.h) &&
                map_.get_label(nb.w) == map_.get_label(nb.h) &&
                map_.get_label(nb.ne) == map_.get_label(nb.h) &&
                map_.get_label(nb.sw) == map_.get_label(nb.h) &&
                map_.get_label(nb.s) == map_.get_label(nb.h) &&
                map_.get_label(nb.se) == map_.get_label(nb.h);
    }

    void jump_west(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi);
    void jump_east(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi);
    void jump_north(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi);
    void jump_south(
        uint32_t from, nbhood_labels nb, double g, double cost, warthog::problem_instance* pi);

    void jump_nw(nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi);
    void jump_ne(nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi);
    void jump_sw(nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi);
    void jump_se(nbhood_labels nb, double g, int successor_set, warthog::problem_instance* pi);

    vl_gridmap& map_;
    warthog::dbword costs_[3];
    wjps_extra* extra_;

    uint8_t* tables_[8];
    vl_gridmap local_map_;
    vl_gridmap_expansion_policy expander_;
    pqueue_min pqueue_;
    nbhood_labels local_nb_;

    static constexpr size_t NBHOOD_TABLE_SIZE = 3 * 3 * 3 *
                                                3 * 3 * 3 *
                                                3 * 3 * 3;
};

}

#endif
