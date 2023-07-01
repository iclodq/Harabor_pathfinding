#ifndef WARTHOG_NBCACHE_H
#define WARTHOG_NBCACHE_H

// nbcache.h
//
// A cache which manages computing and caching neighborhood successor sets in JPSW.
//
// The cache is backed by std::unordered_map and handles rotational symmetry to increase the hit rate.
// In particular, the neighborhood rotated so that the going direction is either north or north-west
// before performing the cache lookup, and the result is rotated back to the required orientation.
//
// @author: Mark Carlson
// @created: 2022-05-20
//

#include "vl_gridmap_expansion_policy.h"
#include "pqueue.h"
#include "labelled_gridmap.h"

namespace warthog
{

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

struct nb_key
{
    // First 8 elements are ordered clockwise starting from the north-west,
    // and the final element is the center location.
    std::array<uint8_t, 9> cells;
    uint8_t diagonal;

    inline bool
    operator==(const nb_key rhs) const
    {
        return cells == rhs.cells && diagonal == rhs.diagonal;
    }
};

}

template<>
struct std::hash<warthog::nb_key>
{
    size_t
    operator()(warthog::nb_key const& k) const
    {
        // jenkins one_at_a_time hash
        size_t h = 0;
        for (int i = 0; i < 9; i++)
        {
            h += k.cells[i];
            h += h << 10;
            h ^= h >> 6;
        }
        h += k.diagonal;
        h += h << 10;
        h ^= h >> 6;
        h += h << 3;
        h ^= h >> 11;
        h += h << 15;
        return h;
    }
};

namespace warthog
{

class nbcache
{
    public:
        nbcache(warthog::cost_table& costs);

        uint8_t
        successors(vl_gridmap& map, nbhood_labels& nb, warthog::jps::direction going);

        inline int
        mem()
        {
            return sizeof(*this) + expander_.mem() + pqueue_.mem();
        }

    private:
        uint8_t
        calculate_successors(uint64_t source);

        vl_gridmap local_map_;
        vl_gridmap_expansion_policy expander_;
        pqueue_min pqueue_;
        nbhood_labels local_nb_;
        std::unordered_map<nb_key, uint8_t> cached_;
};

}

#endif