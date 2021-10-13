#include "search_metrics.h"

std::ostream& operator<<(std::ostream& str, warthog::search_metrics& met)
{
    str
        << " time_elapsed_nano=" << met.time_elapsed_nano_ 
        << " nodes expanded=" << met.nodes_expanded_ 
        << " touched=" << met.nodes_touched_ 
        << " reopened=" << met.nodes_reopen_ 
        << " surplus=" << met.nodes_surplus_
        << " heap-ops=" << met.heap_ops_;
    return str;
}
