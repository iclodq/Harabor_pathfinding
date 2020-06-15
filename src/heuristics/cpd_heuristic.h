#ifndef WARTHOG_CPD_CPD_HEURISTIC_H
#define WARTHOG_CPD_CPD_HEURISTIC_H

// cpd_heuristic.h
//
// Compressed Path Database heuristics compute lower-bound estimates
// by extracting a concrete path between any given pair of nodes.
// If the graph is static the estimate is perfect. If the graph has
// dynamic costs the CPD path is a tentative upperbound solution.
//
// @author: amaheo, dharabor
// @created: 27/02/2020
//

#include "constants.h"
#include "graph_oracle.h"
#include "helpers.h"
#include "xy_graph.h"
#include "cast.h"

#include <stack>

namespace warthog
{

// tracks upper and lower bound costs, and the identity of the first move;
// from a node associated with each entry to a specific node specified
// by ::target_id.
struct cpd_heuristic_cache_entry
{
    cpd_heuristic_cache_entry()
    {
        lb_ = warthog::COST_MAX;
        ub_ = warthog::COST_MAX;
        target_id_ = warthog::SN_ID_MAX;
    }

    warthog::cost_t lb_;
    warthog::cost_t ub_;
    warthog::graph::edge* fm_;
    warthog::sn_id_t target_id_;
};

class cpd_heuristic
{
    typedef std::pair<warthog::sn_id_t, warthog::graph::edge*> stack_pair;
    public:
        // `label_as_lb` switches ub/lb fields.
        cpd_heuristic(warthog::cpd::graph_oracle* cpd, double hscale=1.0,
            bool label_as_lb=false)
            : cpd_(cpd), hscale_(hscale), label_as_lb_(label_as_lb)
        {
            cache_.resize(cpd_->get_graph()->get_num_nodes());
            stack_.reserve(4096);
        }

        ~cpd_heuristic() { }

        inline void
        set_hscale(double hscale)
        { hscale_ = hscale; }

        inline double
        get_hscale()
        { return hscale_; }

        warthog::cpd::graph_oracle*
        get_oracle()
        { return cpd_; }

        // @return a lowerbound cost from node @param start_id to
        // node @param target_id. If no such bound has been established
        // compute one using the CPD in time O(n log(k)) where n is
        // the number of steps to the target and k is the max length of
        // any run-length encoded row in the CPD.
        inline warthog::cost_t
        h(warthog::sn_id_t start_id, warthog::sn_id_t target_id)
        {
            stack_.empty();
            warthog::cost_t lb = 0;
            warthog::cost_t ub = 0;

            // extract
            uint32_t c_id = start_id;
            while(c_id != target_id)
            {
                if(cache_.at(c_id).target_id_ == target_id)
                {
                    // stop when the rest of the path is in cache
                    lb = cache_.at(c_id).lb_;
                    ub = cache_.at(c_id).ub_;
                    break;
                }

                uint32_t move_id =  cpd_->get_move(c_id, target_id);

                warthog::graph::node* cur = cpd_->get_graph()->get_node(c_id);
                warthog::graph::edge* fm = cur->outgoing_begin() + move_id;
                stack_.push_back(stack_pair(c_id, fm));
                c_id = fm->node_id_;
                assert(move_id < cur->out_degree());
            }

            // update the cache
            while(stack_.size())
            {
                stack_pair sp = stack_.back();
                stack_.pop_back();

                warthog::graph::edge_cost_t label =
                    warthog::graph::EDGE_COST_MAX;

                if ((sp.second)->label_ == UINTPTR_MAX)
                {
                    // You probably don't want to be playing with unset labels
                    assert(!label_as_lb_);
                }
                else
                {
                    label = cpd::label_to_wt((sp.second)->label_);
                }

                if (label_as_lb_)
                {
                    // Always apply `hscale_` to the lb
                    lb += label * hscale_;
                    ub += (sp.second)->wt_;
                }
                else
                {
                    lb += (sp.second)->wt_ * hscale_;
                    // Technically does not need the cast
                    ub += (sp.second)->label_;
                }

                cache_.at(sp.first).lb_ = lb;
                cache_.at(sp.first).ub_ = ub;
                cache_.at(sp.first).fm_ = sp.second;
                cache_.at(sp.first).target_id_ = target_id;
            }

            return lb;
        }

        inline void
        h(warthog::sn_id_t start_id, warthog::sn_id_t target_id,
          warthog::cost_t &lower, warthog::cost_t &upper)
        {
            lower = h(start_id, target_id);
            upper = ub(start_id, target_id);
        }

        // @return an upperbound cost from node @param start_id to
        // node @param target_id. if no such bound has been established
        // return instead warthog::COST_MAX
        inline warthog::cost_t
        ub(warthog::sn_id_t start_id, warthog::sn_id_t target_id)
        {
            if(cache_.at(start_id).target_id_ != target_id)
            {
                h(start_id, target_id);
            }
            return cache_.at(start_id).ub_;
        }

        // return the cached first move for the node specified by
        // @param from_id to the node specified by @param target_id
        // @return the first edge on the path if one is cached, else
        // returns 0.
        warthog::sn_id_t
        get_move(warthog::sn_id_t from_id, warthog::sn_id_t target_id)
        {
            if(cache_.at(from_id).target_id_ == target_id)
            {
                cpd_heuristic_cache_entry & entry = cache_.at(from_id);
                return entry.fm_->node_id_;
            }
            return warthog::SN_ID_MAX;
        }

        inline size_t
        mem()
        {
            return
                cpd_->mem() +
                sizeof(warthog::cpd_heuristic_cache_entry)*cache_.capacity() +
                sizeof(stack_pair) * stack_.size();
        }

    private:
        warthog::cpd::graph_oracle* cpd_;
        double hscale_;
        bool label_as_lb_;
        std::vector<warthog::cpd_heuristic_cache_entry> cache_;
        std::vector<stack_pair> stack_;
};

}

#endif