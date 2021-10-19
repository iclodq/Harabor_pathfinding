#ifndef WARTHOG_SOLUTION_H
#define WARTHOG_SOLUTION_H

// a wrapper for solutions found by algorithms implemented
// in the warthog library
//
// @author: dharabor
// @created: 2017-05-03
//

#include "search_metrics.h"
#include "search_node.h"

#include <vector>
#include <ostream>
#include "constants.h"

namespace warthog
{

class solution
{
    public:

        solution()
        { 
            reset (); 
            path_.reserve(1024); 
        }

        solution(const solution& other) :   
            met_(other.met_), path_(other.path_)
        { }

        inline void
        print_metrics(std::ostream& out)
        {
            out << "sum_of_edge_costs=" << sum_of_edge_costs_;
            out << met_;
        }

        inline void
        print_path(std::ostream& out)
        {
            out << "path=";
           for(auto &state : path_) { out << state << " "; }
        }

        void
        reset()
        {
            s_node_ = 0;
            sum_of_edge_costs_ = warthog::COST_MAX;
            path_.clear();
            met_.reset();
        }

        friend std::ostream& operator<<(
                std::ostream& str, warthog::solution& sol);

        // search performance metrics 
        warthog::search_metrics met_;

        // the solution itself. we store the incumbent node 
        // which produced the solution and the concrete path,
        // from start to target
        warthog::search_node* s_node_;
        warthog::cost_t sum_of_edge_costs_;
        std::vector<warthog::sn_id_t> path_;
};

}

std::ostream& operator<<(std::ostream& str, warthog::solution& sol);

#endif
