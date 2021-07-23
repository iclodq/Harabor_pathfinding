#include "road_heuristic.h"

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::SPHERICAL>::distance_(
    double lon_s, double lat_s, double lon_t, double lat_t)
{
    return warthog::geo::spherical_distance(lon_s, lat_s, lon_t, lat_t);
}

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::GREAT_CIRCLE>::distance_(
    double lon_s, double lat_s, double lon_t, double lat_t)
{
    return warthog::geo::great_circle_distance(lon_s, lat_s, lon_t, lat_t);
}

template<>
warthog::cost_t
warthog::road_heuristic_base<warthog::geo::VINCENTY>::distance_(
    double lon_s, double lat_s, double lon_t, double lat_t)
{
    return warthog::geo::vincenty_distance(lon_s, lat_s, lon_t, lat_t);
}
