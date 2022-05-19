#ifndef NBCACHE_H
#define NBCACHE_H

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

class nbcache
{
public:
    nbcache();

    warthog::jps::direction successors(
        vl_gridmap& map, nbhood_labels& nb, warthog::jps::direction going);

    inline int mem() {
        return sizeof(*this) + expander_.mem() + pqueue_.mem();
    }

private:
    warthog::jps::direction calculate_successors(uint32_t source);

    vl_gridmap local_map_;
    vl_gridmap_expansion_policy expander_;
    pqueue_min pqueue_;
    nbhood_labels local_nb_;
};

}

#endif