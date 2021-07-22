#include "haversine_heuristic.h"
#include "xy_graph.h"
#include "geography.h"
#include <cmath>

#define Pi 0.017453292519943295 //#Pi/180

warthog::haversine_heuristic::haversine_heuristic(warthog::graph::xy_graph* g)
{
    g_ = g;
    // The heuristic scale factor, tuned for the DIMACS instances to guarantee
    // admissibility. Still admissible with 0.999 but 0.9988 is safe.
    hscale_ = 0.9988;
}

warthog::haversine_heuristic::~haversine_heuristic() = default;

double
warthog::haversine_heuristic::h(warthog::sn_id_t id, warthog::sn_id_t id2)
{
    int32_t x, x2;
    int32_t y, y2;
    g_->get_xy((uint32_t)id, x, y);
    g_->get_xy((uint32_t)id2, x2, y2);

    return hscale_ * warthog::haversine_heuristic::h(
        x / warthog::geo::DIMACS_RATIO, y / warthog::geo::DIMACS_RATIO,
        x2 / warthog::geo::DIMACS_RATIO, y2 / warthog::geo::DIMACS_RATIO);
}

double
warthog::haversine_heuristic::h(
    double lon1, double lat1, double lon2, double lat2)
{
    // NB: precision loss when warthog::cost_t is an integer

    double a = 0.5 - cos((lat2 - lat1) * Pi) / 2
        + cos(lat1 * Pi) * cos(lat2 * Pi) * (1 - cos((lon2 - lon1) * Pi)) / 2;

    // 2*R*asin... converted to m from km then converted to 0.1m;
    return floor(127420000 * asin(sqrt(a)));
}

void
warthog::haversine_heuristic::set_hscale(double hscale)
{
    if(hscale > 0)
    {
        hscale_ = hscale;
    }
}

double
warthog::haversine_heuristic::get_hscale()
{
    return hscale_;
}

size_t
warthog::haversine_heuristic::mem()
{
    return sizeof(*this);
}
