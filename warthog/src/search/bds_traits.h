#ifndef WARTHOG_BDS_TRAITS
#define WARTHOG_BDS_TRAITS

// src/search/bds_traits.h
//
// Traits that specify how a Bi-Directional Search should behave:
//  - with respect to lower-bound comptation
//  - with respect to determining failure
//
//  @author: dharabor
//  @created 2021-10-09
//

namespace warthog
{

// We distinguish between and specialise for three types of BDS:
//  - bi-directional heuristic search (BHS).
//  - bi-directional dijkstra (BDS).
//  - bi-directional contraction hierarchies (BCH).
enum BDS_ALGO
{
BHS, 
BDIJK, 
BCH  
};


// Default behaviours for Bidirectional Search
template<BDS_ALGO A>
struct bds_traits
{
    // One of the ways in which bi-directional search algorithms 
    // differ is how to compute a lower-bound on the cost-to-go.
    // The default strategy is to take the minimum f-value in 
    // either direction. This is the approach used by Bidirectional A*
    // and is safe for any admissible heuristic. For the special case
    // of h=0, we can do better and this is handled via specialisation
    // 
    // @param ftop: the most promising node in the forward queue
    // @param btop: the most promising node in the backward queue
    // @return: the minimum f-value among ftop and btop
    static inline warthog::cost_t 
    lowerbound(warthog::search_node* ftop, warthog::search_node* btop)
    {
        return std::min(
                ftop ? ftop->get_f() : warthog::COST_MAX, 
                btop ? btop->get_f() : warthog::COST_MAX);
    }

    // Another way in which bi-directional search algorithms can
    // differ is how to decide failure; i.e., how to determine that no 
    // solution exists. The default strategy is to return failure
    // when either of the priority queues is exhausted. This is the approach 
    // used by Bi-directional Dijkstra and Bi-directional A* but it is
    // not safe for other algorithms, such as Contraction Hierarchies.
    // We handle those cases via specialisation.
    // 
    // @param fqueue_size: the size of the forward queue
    // @param bqueue_size: the size of the backward queue
    // @return true only if either queue size is zero, else false.
    static inline bool
    solvable(uint32_t fqueue_size, uint32_t bqueue_size)
    {
        return (fqueue_size && bqueue_size);
    }
};

// Bi-directional Dijkstra search can take as an admissible lower-bound 
// the sum of the minimum f-values on the two open lists.
template<>
inline warthog::cost_t 
warthog::bds_traits<warthog::BDS_ALGO::BDIJK>::lowerbound(
        warthog::search_node* ftop, warthog::search_node* btop)
{
    assert(ftop && btop);
    return ftop->get_f() + btop->get_f();
}

//  BCH terminates with failure when both OPEN lists are exhausted. 
//  This is because the search only considers a subset of edges when 
//  expanding a node in a given direction: outgoing up edges only in the 
//  forward direction and incoming up edges only in the backward direction.  
template<>
inline bool 
warthog::bds_traits<warthog::BDS_ALGO::BCH>::solvable(
        uint32_t fqueue_size, uint32_t bqueue_size)
{
    return (fqueue_size || bqueue_size);
}

}

#endif

