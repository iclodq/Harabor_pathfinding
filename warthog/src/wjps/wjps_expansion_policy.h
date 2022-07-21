#ifndef WJPS_EXPANSION_POLICY_H
#define WJPS_EXPANSION_POLICY_H

#include "jps.h"
#include "expansion_policy.h"
#include "labelled_gridmap.h"
#include "nbcache.h"
#include "cost_table.h"

namespace warthog
{

struct wjps_extra;

class wjps_expansion_policy : public expansion_policy
{
public:
    /*
    * This will modify the provided map, changing nonzero labels to 1 for one tile type and 2 for
    * the other (the expansion policy will remember the costs).
    *
    * If there are more than 2 nonzero tile types, an exception is thrown.
    */
    wjps_expansion_policy(nbcache& nbcache, vl_gridmap& map, cost_table& costs);
    ~wjps_expansion_policy();

    void fill_nb_cache();
    void fill_jump_cache();

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

    virtual size_t mem();

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

    wjps_extra& get_extra(uint32_t id, warthog::problem_instance* pi);

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
                map_.get_label(nb.e) == map_.get_label(nb.h) &&
                map_.get_label(nb.sw) == map_.get_label(nb.h) &&
                map_.get_label(nb.s) == map_.get_label(nb.h) &&
                map_.get_label(nb.se) == map_.get_label(nb.h);
    }

    template<int SLOT, typename F>
    void calculate_jump(uint32_t start, int delta, uint32_t version, F cost);

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
    cost_table& costs_;
    wjps_extra* extra_;
    nbcache& nbcache_;
    uint32_t* row_versions_;
    uint32_t* col_versions_;
};

}

#endif
