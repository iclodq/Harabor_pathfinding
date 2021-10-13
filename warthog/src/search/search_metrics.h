#ifndef WARTHOG_SEARCH_METRICS_H
#define WARTHOG_SEARCH_METRICS_H

// search/search_metrics.h
//
// A container for all the metrics that are recorded when running a search
//
// @author: dharabor
// @created: 2021-10-13
//

#include <iostream>

namespace warthog
{

struct search_metrics
{
    search_metrics() { reset(); }

    warthog::search_metrics& 
    operator=(const warthog::search_metrics& other)
    {
        time_elapsed_nano_ = other.time_elapsed_nano_;
        nodes_expanded_ = other.nodes_expanded_;
        nodes_touched_ = other.nodes_touched_;
        nodes_surplus_ = other.nodes_surplus_;
        nodes_reopen_ = other.nodes_reopen_;
        heap_ops_ = other.heap_ops_;
        return *this;
    }

    inline void
    reset()
    {
        time_elapsed_nano_ = 0;
        nodes_expanded_ = 0; 
        nodes_touched_= 0;
        nodes_surplus_ = 0;
        nodes_reopen_ = 0;
        heap_ops_ = 0;
    }
    
    double time_elapsed_nano_;
    uint32_t nodes_expanded_;
    uint32_t nodes_touched_;
    uint32_t nodes_surplus_;
    uint32_t nodes_reopen_;
    uint32_t heap_ops_;

    //friend std::ostream& operator<<(std::ostream& str, warthog::search_metrics& met);
};

}

std::ostream& operator<<(std::ostream& str, warthog::search_metrics& met);

#endif
