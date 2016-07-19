#ifndef WARTHOG_NODE_FILTER_H
#define WARTHOG_NODE_FILTER_H

// search/node_filter.h
// 
// Implement this class to filter or prune nodes during
// search.
//
// @author: dharabor
// @created: 2016-01-26

#include <cstdint>
namespace warthog
{

class node_filter
{
    public:
        virtual 
        bool filter(uint32_t) = 0;

        uint32_t
        mem() { return 0; }
};

}

#endif
