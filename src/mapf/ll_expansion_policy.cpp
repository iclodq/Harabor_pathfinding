#include "cbs.h"
#include "grid.h"
#include "helpers.h"
#include "ll_expansion_policy.h"
#include "problem_instance.h"
#include "time_constraints.h"

#include <algorithm>

using namespace warthog::cbs;

warthog::ll_expansion_policy::ll_expansion_policy(
		warthog::gridmap* map, warthog::cbs_ll_heuristic* h) : map_(map), h_(h)
{
    neis_ = new warthog::arraylist<neighbour_record>(32);

    map_xy_sz_ = map->height() * map->width();
    assert(map_xy_sz_ > 0);

    cons_ = new warthog::mapf::time_constraints<warthog::mapf::cell_constraint>
             (map_xy_sz_);

    // preallocate memory for search nodes up to some number of timesteps 
    // in advance. for subsequent timesteps memory is allocated
    // dynamically
    search_node_pool_ = new std::vector<warthog::mem::node_pool*>();
    for(uint32_t i = 0; i < 128; i++)
    {
        search_node_pool_->push_back(new warthog::mem::node_pool(map_xy_sz_));
    }
}

warthog::ll_expansion_policy::~ll_expansion_policy()
{
    for(uint32_t i = 0; i < search_node_pool_->size(); i++)
    {
        delete search_node_pool_->at(i);
    }
    search_node_pool_->clear();
    delete search_node_pool_;
    delete cons_;
    delete neis_;
}

void
warthog::ll_expansion_policy::get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y)
{
    map_->to_unpadded_xy((uint32_t)nid, (uint32_t&)x, (uint32_t&)y);
}

warthog::search_node* 
warthog::ll_expansion_policy::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->start_id_);
    uint32_t timestep = (uint32_t)(pi->start_id_ >> 32);
    return __generate(padded_id, timestep);
}

warthog::search_node*
warthog::ll_expansion_policy::generate_target_node(
        warthog::problem_instance* pi)
{
    auto found_target = h_->set_current_target(pi->target_id_);
    if (!found_target)
    {
        h_->compute_new_h_value(pi->target_id_, map_);
        found_target = h_->set_current_target(pi->target_id_);
        assert(found_target);
    }
    uint32_t padded_id = map_->to_padded_id((uint32_t)pi->target_id_);
    uint32_t timestep = (uint32_t)(pi->target_id_ >> 32);
    return __generate(padded_id, timestep);
}

size_t
warthog::ll_expansion_policy::mem()
{
   size_t total = sizeof(*this) + map_->mem();
   size_t tm_sz = search_node_pool_->size();
   for(uint32_t i = 0; i < tm_sz; i++)
   {
       total += search_node_pool_->at(i)->mem();
   }
   total += sizeof(neighbour_record) * neis_->capacity();
   return total;
}
