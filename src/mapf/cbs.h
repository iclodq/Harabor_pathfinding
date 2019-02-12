#ifndef WARTHOG_CBS_H
#define WARTHOG_CBS_H

// mapf/cbs.h
//
// Utilities for working with Conflict-based Search.
//
// @author: dharabor
// @created: 2018-11-06
//

#include "grid.h"
#include "gridmap.h"
#include "pqueue.h"
#include "reservation_table.h"
#include "search_node.h"

namespace warthog
{

namespace cbs
{

enum move 
{
    NORTH = __builtin_ffs(warthog::grid::NORTH)-1,
    SOUTH = __builtin_ffs(warthog::grid::SOUTH)-1,
    EAST = __builtin_ffs(warthog::grid::EAST)-1,
    WEST = __builtin_ffs(warthog::grid::WEST)-1,
    WAIT = WEST + 1,
    INVALID = 5
};

// This data structure describes the constraints CBS imposes on 
// single grid cells. A brief description of the members:
//
// timestep_ is the time at which the constraints apply
// v_ is true if the cell is constrained / blocked
// e_ indicates which of the 8 outgoing directions are constrained / blocked
struct cbs_constraint
{
    cbs_constraint() 
        : timestep_(0), v_(0), e_(0) { }

    cbs_constraint(uint16_t timestep, uint8_t v, uint8_t e) 
        : timestep_(timestep), v_(v), e_(e) { }

    uint16_t timestep_; 
    uint8_t v_;
    uint8_t e_;
};

// this data structure defines the lessthan comparator for 
// CBS low-level search. Tie-breaks as follows:
// 1. smaller f-value, then
// 2. larger g-value and !is_reserved, then
// 3. !is_reserved, then
// 4. larger g-value
class cmp_cbs_ll_lessthan
{
    public:
        cmp_cbs_ll_lessthan(warthog::reservation_table* restab)
            : restab_(restab)
        { 
            is_reserved_fn_ = &cmp_cbs_ll_lessthan::__is_reserved;
        }

        bool
        operator()(const warthog::search_node& first, const warthog::search_node& second)
        {
            const auto first_is_reserved = (this->*(is_reserved_fn_))(first.get_id());
            const auto second_is_reserved = (this->*(is_reserved_fn_))(second.get_id());
            return (first.get_f() <  second.get_f()) ||
                   (first.get_f() == second.get_f() && first_is_reserved <  second_is_reserved) ||
                   (first.get_f() == second.get_f() && first_is_reserved == second_is_reserved && first.get_g() >  second.get_g()) ||
                   (first.get_f() == second.get_f() && first_is_reserved == second_is_reserved && first.get_g() == second.get_g() && static_cast<bool>(rand() % 2));
        }
    
    private:
        typedef bool(cmp_cbs_ll_lessthan::*fn_is_reserved)(warthog::sn_id_t time_indexed_id);

        warthog::reservation_table* restab_;
        fn_is_reserved  is_reserved_fn_;

        bool
        __is_reserved_dummy(warthog::sn_id_t) { return false; }

        bool
        __is_reserved(warthog::sn_id_t map_id)
        { 
            return restab_->is_reserved(map_id);
        }
};

typedef pqueue<cmp_cbs_ll_lessthan> pqueue_cbs_ll;

}

}

#endif
