#ifndef WARTHOG_MAPF_TIME_CONSTRAINTS
#define WARTHOG_MAPF_TIME_CONSTRAINTS

// mapf/time_constraints.h
//
// This data structure describes the all constraints that currently apply
// to an associated grid map. A constraint describes: 
//  (i) the cost of reaching the cell (i.e. a cost on the vertex)
//  (ii) a set of edge labels that describe the costs of move actions 
//  applicable  in the cell (once reached) 
//  (iii) the timestep when the constraint applies
// 
// @author: dharabor
// @created: 2018-12-06
//

namespace warthog
{
namespace mapf
{

// The constraint data type.
//
// v_: indicates if the tile is blocked or not
// the default state is unblocked
//
// e_: indicates the cost of each move action.
// the default cost for each action is 1. 
// the special value warthog::INF indicates the move cannot be executed.
// 
// timestep_: indicates the timestep when the edge costs apply. 
// the default timestep is 0.
// 
struct cell_constraint
{
    double e_[5]{1,1,1,1,1};
    uint16_t timestep_{0};
};

static_assert(std::is_trivially_copyable<cell_constraint>::value);

template<typename CONSTRAINT>
class time_constraints
{
    public:

       time_constraints() noexcept = delete;
       time_constraints(const uint32_t map_xy_sz) noexcept : cons_(map_xy_sz) {}
       ~time_constraints() noexcept = default;

       time_constraints(const time_constraints& other) = default;
       time_constraints(time_constraints&& other) noexcept = default;
       time_constraints& operator=(const time_constraints& other) = default;
       time_constraints& operator=(time_constraints&& other) noexcept = default;

       // get size
       inline uint32_t
       map_xy_sz() const
       {
           return cons_.size();
       }

       // add a constraint to location @param xy_id
       inline void
       add_or_replace_constraint(const uint32_t xy_id, const CONSTRAINT& con)
       {
           auto& xy_cons = cons_[xy_id];
           auto it = find_pos(xy_id, con.t);
           if (it != xy_cons.end() && it->timestep_ == con.timestep_)
           {
               *it = con;
           }
           else
           {
               xy_cons.insert(it, con);
           }
       }

       // return all constraints associated with the xy location
       // @param padded_id
       inline std::vector<CONSTRAINT>&
       get_constraint_set(const uint32_t xy_id)
       {
#ifndef NDEBUG
           assert(xy_id < map_xy_sz());
#endif
           return cons_[xy_id];
       }

       // return the constraint associated with the location
       // @param xy_id, at the time @param timestep
       inline CONSTRAINT*
       get_constraint(uint32_t xy_id, uint32_t timestep)
       {
           CONSTRAINT* con = nullptr;

           auto& xy_cons = cons_[xy_id];
           auto it = find_pos(xy_id, timestep);
           if (it != xy_cons.end() && it->timestep_ == timestep)
               con = &*it;

           return con;
       }

       // create or return the constraint associated with the location
       // @param xy_id, at the time @param timestep
       inline CONSTRAINT&
       get_or_create_constraint(uint32_t xy_id, uint32_t timestep)
       {
           CONSTRAINT* con;

           auto& xy_cons = cons_[xy_id];
           auto it = find_pos(xy_id, timestep);
           if (it != xy_cons.end() && it->timestep_ == timestep)
           {
               con = &*it;
           }
           else
           {
               con = &*xy_cons.emplace(it);
               con->timestep_ = timestep;
           }

           return *con;
       }

       // remove all constraints associated with location
       inline void
       clear_constraint_at(const uint32_t xy_id)
       {
           auto& xy_cons = cons_[xy_id];
           xy_cons.clear();
       }

       // remove all constraints
       inline void
       clear()
       {
           for (auto& xy_cons : cons_)
               xy_cons.clear();
       }

       // remove all constraints with default values
       void
       clean_up()
       {
           for (auto& xy_cons : cons_)
           {
               bool deleted = false;
               for (auto it = xy_cons.begin(); it != xy_cons.end();)
               {
                   auto& con = *it;
                   if (con.e_[warthog::cbs::move::NORTH] == 1 &&
                       con.e_[warthog::cbs::move::SOUTH] == 1 &&
                       con.e_[warthog::cbs::move::EAST] == 1 &&
                       con.e_[warthog::cbs::move::WEST] == 1 &&
                       con.e_[warthog::cbs::move::WAIT] == 1)
                   {
                       std::swap(*it, xy_cons.back());
                       xy_cons.pop_back();
                       deleted = true;
                   }
                   else
                   {
                       ++it;
                   }
               }
               if (deleted)
               {
                   std::sort(xy_cons.begin(),
                             xy_cons.end(),
                             [](const CONSTRAINT& a, const CONSTRAINT& b)
                             { return a.timestep_ < b.timestep_; });
               }
           }
       }

    private:
        std::vector<std::vector<CONSTRAINT>> cons_;

        inline auto
        find_pos(const uint32_t xy_id, const uint16_t t)
        {
#ifndef NDEBUG
            assert(xy_id < map_xy_sz());
#endif
            auto& xy_cons = cons_[xy_id];
            auto it = std::lower_bound(xy_cons.begin(),
                                       xy_cons.end(),
                                       t,
                                       [](const CONSTRAINT& con, const uint16_t t)
                                       {
                                           return con.timestep_ < t;
                                       });
            return it;
        }

}; // time_constraints


} // mapf
} // warthog

#endif

