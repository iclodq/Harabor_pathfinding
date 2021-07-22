#ifndef WARTHOG_HAVERSINE_HEURISTIC_H
#define WARTHOG_HAVERSINE_HEURISTIC_H

// haversine_heuristic.h
//
// Haversine (Great-Circle) Heuristic for measuring geodesic distances on
// earth.
//
// @author: sahamdi
// @updated: 05/06/21
//

#include "constants.h"
#include "forward.h"

namespace warthog
{

typedef void (*xyFn)(uint32_t id, int32_t& x, int32_t& y);
class haversine_heuristic
{
  public:
    explicit haversine_heuristic(warthog::graph::xy_graph* g);
    ~haversine_heuristic();

    double
    h(warthog::sn_id_t id, warthog::sn_id_t id2);

    static double
    h(double x, double y, double x2, double y2);

    void
    set_hscale(double hscale);

    double
    get_hscale();

    size_t
    mem();

  private:
    warthog::graph::xy_graph* g_;
    double hscale_;
};

}

#endif
