#ifndef WARTHOG_RESERVATION_TABLE_H
#define WARTHOG_RESERVATION_TABLE_H

// mapf/reservation_table.h
//
// A reservation table that describes a solution to a
// MAPF problem. Each cell that appears on the path
// of an agent is "marked" in the reservation table.
// The idea is to share this information among agents so
// they can plan without colliding into one another.
//
// This implementation uses a single bit to represent each
// cell in a time-expanded grid graph.
//
// For more details see: 
// Sharon, Guni, et al. "Conflict-based search for optimal multi-agent pathfinding." 
// Artificial Intelligence 219 (2015): 40-66.
//
// @author: dharabor
// @created: 2018-11-05
//

#include "constants.h"
#include "cpool.h"

#include <stdint.h>
#include <vector>
#include <cstring>
#include <cmath>

namespace warthog
{

class reservation_table
{
    public:
        reservation_table(const uint32_t map_sz) :
            table_(reinterpret_cast<char*>(
                        std::malloc((4 * sqrt(map_sz)) * map_sz / CHAR_BIT + 1))),
            timesteps_(4 * sqrt(map_sz)),
            map_sz_(map_sz)
        {
            clear_reservations();
        }
        ~reservation_table() noexcept
        {
            std::free(table_);
        }

        inline bool
        is_reserved(const uint32_t xy_id, const int32_t timestep)
        {
            if (timestep >= timesteps_)
            {
                return false;
            }

            const auto elem = timestep * map_sz_ + xy_id;
            const auto idx = elem / CHAR_BIT;
            const char mask = 0x1 << (elem % CHAR_BIT);
            return (table_[idx] & mask) != 0x0;
        }

        inline bool
        is_reserved(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = time_indexed_map_id >> 32;
            uint32_t xy_id = time_indexed_map_id & UINT32_MAX;
            return is_reserved(xy_id, timestep);
        }

        inline void
        reserve(const uint32_t xy_id, const int32_t timestep)
        {
            assert(xy_id < map_sz_);

            if (timestep >= timesteps_)
            {
                const auto new_timesteps = timestep + 50;
                table_ = reinterpret_cast<char*>(
                    std::realloc(table_, new_timesteps * map_sz_ / CHAR_BIT + 1));
                if (!table_)
                {
                    printf("Failed to reallocate memory for reservation table");
                    std::abort();
                }
                timesteps_ = new_timesteps;
            }

            const auto elem = timestep * map_sz_ + xy_id;
            const auto idx = elem / CHAR_BIT;
            const char mask = 0x1 << (elem % CHAR_BIT);
            table_[idx] |= mask;
        }

        inline void
        reserve(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = time_indexed_map_id >> 32;
            uint32_t xy_id = time_indexed_map_id & UINT32_MAX;
            reserve(xy_id, timestep);
        }

        inline void
        unreserve(const uint32_t xy_id, const int32_t timestep)
        {
            assert(xy_id < map_sz_);
            assert(timestep < timesteps_);

            const auto elem = timestep * map_sz_ + xy_id;
            const auto idx = elem / CHAR_BIT;
            const char mask = 0x1 << (elem % CHAR_BIT);
            table_[idx] &= ~mask;
        }

        inline void
        unreserve(warthog::sn_id_t time_indexed_map_id)
        {
            uint32_t timestep = (uint32_t)(time_indexed_map_id >> 32);
            uint32_t xy_id = (uint32_t)(time_indexed_map_id & UINT32_MAX);
            unreserve(xy_id, timestep);
        }

        inline void
        clear_reservations()
        {
            memset(table_, 0, timesteps_ * map_sz_ / CHAR_BIT + 1);
        }

    private:
        char* table_;
        int32_t timesteps_;
        const uint32_t map_sz_;

};

}

#endif
