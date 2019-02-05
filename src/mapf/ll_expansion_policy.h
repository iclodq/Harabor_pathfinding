#ifndef WARTHOG_LL_EXPANSION_POLICY_H
#define WARTHOG_LL_EXPANSION_POLICY_H

// mapf/ll_expansion_policy.h
//
// An expansion policy for low-level search on uniform-cost manhattan grids.
// In this policy constraints are applied only on edges.

// At each location there are two different types of actions:
// - cardinal moves (at a cost of 1)
// - wait moves (at a cost of 1)
//
// Each action, except wait, moves the agent from its current grid
// location to an adjacent grid location. Each action (including wait)
// advances time by one time-step.
//
// @author: dharabor
// @created: 2018-11-01
//

#include "cbs_ll_heuristic.h"
#include "expansion_policy.h"
#include "forward.h"
#include "gridmap.h"
#include "search_node.h"
#include "time_constraints.h"

#include <memory>

namespace warthog
{

typedef union
{
    uint64_t t_id;
    struct
    {
        uint32_t id : 32;
        uint32_t t : 32;
    };
} packed_time_and_id;

class ll_expansion_policy 
{
	public:
		ll_expansion_policy(
                warthog::gridmap* map, 
                warthog::cbs_ll_heuristic* h);

		~ll_expansion_policy();

		inline void
		reset()
		{
			current_ = 0;
            neis_->clear();
		}

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            current_ = 0;
            n(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(current_ < neis_->size())
            {
                ret = (*neis_)[current_].first;
                cost = (*neis_)[current_].second;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            current_++;
            n(ret, cost);
		}

        template<bool goal_has_timestep>
		void
        expand(warthog::search_node* current, warthog::problem_instance* problem)
        {
            reset();

            // get the xy id of the current node and extract current timestep
            const packed_time_and_id xyt{.t_id = current->get_id()};

            if constexpr (goal_has_timestep)
            {
                const packed_time_and_id goal{.t_id = problem->target_id_};
                if (xyt.t == goal.t)
                    return;
                assert(xyt.t < goal.t);
            }

            // neighbour ids are calculated using xy_id offsets
            uint32_t nid_m_w = xyt.id - map_->width();
            uint32_t nid_p_w = xyt.id + map_->width();

            // edge constraints for the current node
            warthog::mapf::cell_constraint* cur_cc =
                cons_->get_constraint(xyt.id, xyt.t);

            // move NORTH
            double move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::NORTH] : 1;
            if( map_->get_label(nid_m_w) && move_cost != warthog::INF32 )
            {
                add_neighbour( __generate(nid_m_w, xyt.t+1), move_cost );
            }

            // move EAST
            move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::EAST] : 1;
            if( map_->get_label(xyt.id + 1) && move_cost != warthog::INF32)
            {
                add_neighbour( __generate(xyt.id+1, xyt.t+1), move_cost );
            }

            // move SOUTH
            move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::SOUTH] : 1;
            if( map_->get_label(nid_p_w) && move_cost != warthog::INF32 )
            {
                add_neighbour( __generate(nid_p_w, xyt.t+1), move_cost );
            }

            // move WEST
            move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::WEST] : 1;
            if( map_->get_label(xyt.id - 1) && move_cost != warthog::INF32 )
            {
                add_neighbour( __generate(xyt.id-1, xyt.t+1), move_cost );
            }

            // move WAIT
            move_cost = cur_cc ? cur_cc->e_[warthog::cbs::move::WAIT] : 1;
            if( move_cost != warthog::INF32 )
            {
                add_neighbour( __generate(xyt.id, xyt.t+1), move_cost );
            }
        }

        void
        get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y);

        warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);
        
        warthog::search_node*
        generate(warthog::sn_id_t node_id)
        {
            return __generate((uint32_t)node_id, node_id>>32);
        }

        uint32_t get_safe_arrival_time(warthog::problem_instance* pi)
        {
            const packed_time_and_id goal{.t_id = pi->target_id_};

            const auto& goal_cons = cons_->get_constraint_set(goal.id);
            for (int32_t idx = goal_cons.size() - 1; idx >= 0; --idx)
                if (goal_cons[idx].e_[warthog::cbs::move::NORTH] < 0 ||
                    goal_cons[idx].e_[warthog::cbs::move::SOUTH] < 0 ||
                    goal_cons[idx].e_[warthog::cbs::move::EAST] < 0 ||
                    goal_cons[idx].e_[warthog::cbs::move::WEST] < 0)
                {
                    return goal_cons[idx].timestep_ + 1;
                }
            return 0;
        }

        inline bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            // same location
            const packed_time_and_id xyt{.t_id = n->get_id()};
            const packed_time_and_id goal{.t_id = pi->target_id_};
            return xyt.id == goal.id;
        }

        inline bool
        is_target_time_eq(warthog::search_node* n, warthog::problem_instance* pi)
        {
            // same location and time
            return n->get_id() == pi->target_id_;
        }

        inline bool
        is_target_time_geq(warthog::search_node* n, warthog::problem_instance* pi)
        {
            // same location
            const packed_time_and_id xyt{.t_id = n->get_id()};
            const packed_time_and_id goal{.t_id = pi->target_id_};
            return xyt.id == goal.id && xyt.t >= goal.t;
        }

        warthog::mapf::time_constraints<warthog::mapf::cell_constraint>*
        get_time_constraints() { return cons_; }

		size_t 
        mem();

	
	private:
        typedef std::pair<warthog::search_node*, double> neighbour_record;

		warthog::gridmap* map_;
        uint32_t map_xy_sz_;
        warthog::cbs_ll_heuristic* h_;

        std::vector<warthog::mem::node_pool*>* search_node_pool_;
        warthog::mapf::time_constraints<warthog::mapf::cell_constraint>* cons_;

        arraylist<neighbour_record>* neis_;
        uint32_t current_;

        inline warthog::search_node* 
        __generate(uint32_t xy_id, uint32_t timestep)
        {
            while(timestep >= search_node_pool_->size())
            {
                search_node_pool_->push_back(
                            new warthog::mem::node_pool(
                                map_->height() * map_->width()));
            }
            warthog::search_node* nei = search_node_pool_->at(timestep)->generate(xy_id);
            nei->set_id((((warthog::sn_id_t)timestep) << 32) | xy_id);
            return nei;
        }


        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
        }

};

}

#endif

