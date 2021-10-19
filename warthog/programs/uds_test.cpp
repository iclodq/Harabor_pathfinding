// programs/roadhog.cpp
//
// Pulls together a variety of different algorithms for
// routing on road graphs.
//
// @author: dharabor
// @created: 2016-11-24
//

#include "anytime_astar.h"
#include "apex_filter.h"
#include "bb_filter.h"
#include "bb_labelling.h"
#include "bch_expansion_policy.h"
#include "bch_bb_expansion_policy.h"
#include "bidirectional_graph_expansion_policy.h"
#include "bidirectional_search.h"
#include "cfg.h"
#include "constants.h"
#include "contraction.h"
#include "cpd_extractions.h"
#include "cpd_graph_expansion_policy.h"
#include "cpd_heuristic.h"
#include "cpd_search.h"
#include "depth_first_search.h"
#include "dimacs_parser.h"
#include "euclidean_heuristic.h"
#include "fch_bb_expansion_policy.h"
#include "fch_expansion_policy.h"
#include "flexible_astar.h"
#include "graph_expansion_policy.h"
#include "graph_oracle.h"
#include "lazy_graph_contraction.h"
#include "problem_instance.h"
#include "solution.h"
#include "timer.h"
#include "unidirectional_search.h"
#include "workload_manager.h"
#include "xy_graph.h"
#include "zero_heuristic.h"

#include "getopt.h"

#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unordered_map>

// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;

// suppress the header row when printing results? (default: no)
int suppress_header = 0;

long nruns = 1;

void
help()
{
    std::cerr
    << "==> manual <==\n"
    << "This program solves point-to-point pathfinding problems on road networks.\n"
    << "Road networks are specified as xy-graphs and collections of instances are \n"
    << "specified as p2pz. Both formats are similar to that used at the 9th DIMACS \n"
    << "challenge. The main differences are: \n"
    << "(i) a single file format (cf. gr/co files); \n"
    << "(ii) node/arc ids are zero indexed (cf. 1-indexed) and; \n"
    << "(iii) the enforcement of the triangle inequality for all arc costs \n"
    << "(cf. not, as was the case at the competition)\n\n"
    << ""
    << "The following are valid program parameters:\n"
    << "\t--alg [ algorithm name (required) ]\n"
    << "\t--input [ algorithm-specific input files (omit to show options) ] \n"
    << "\t--problem [ ss or p2p problem file (required) ]\n"
    << "\t--verbose (print debug info; omitting this param means no)\n"
    << "\t--nruns [int (repeats per instance; default=" << nruns << ")]\n"
    << "\nRecognised values for --alg:\n"
    << "\tastar, astar-bb, dijkstra, bi-astar, bi-dijkstra\n"
    << "\tbch, bch-astar, bch-bb, fch, fch-bb\n"
    << "\tdfs, cpd, cpd-search\n";
}

void
run_experiments( 
        auto& algo_fn,
        std::string alg_name,
        warthog::search_parameters* par, 
        warthog::dimacs_parser* parser, 
        std::ostream& out)
{
    std::cerr << "running experiments\n";
    std::cerr << "(averaging over " << nruns << " runs per instance)\n";

    if(!suppress_header)
    {
        std::cout
            << "id\talg\texp\tgen\treopen\tsurp\theap_op"
            << "\tnanos\tcost\tlen\tmap\n";
    }
    uint32_t exp_id = 0;
    for(auto it = parser->experiments_begin();
            it != parser->experiments_end();
            it++)
    {
        warthog::dimacs_parser::experiment exp = (*it);
        warthog::solution sol;
        warthog::sn_id_t start_id = exp.source;
        warthog::sn_id_t target_id = exp.p2p ? exp.target : warthog::SN_ID_MAX;
        warthog::problem_instance pi(start_id, target_id, verbose);
        uint32_t expanded=0, reopen=0, heap_ops=0, generated=0, surplus=0;
        double nano_time = 0;
        for(uint32_t i = 0; i < nruns; i++)
        {
            sol.reset();
            algo_fn(&pi, par, &sol);

            expanded += sol.met_.nodes_expanded_;
            reopen += sol.met_.nodes_reopen_;
            heap_ops += sol.met_.heap_ops_;
            generated += sol.met_.nodes_generated_;
            surplus += sol.met_.nodes_surplus_;
            nano_time += sol.met_.time_elapsed_nano_;
        }

        out
            << exp_id++ <<"\t"
            << alg_name << "\t"
            << expanded / nruns << "\t"
            << generated / nruns << "\t"
            << reopen / nruns << "\t"
            << surplus / nruns << "\t"
            << heap_ops / nruns << "\t"
            << nano_time / nruns << "\t"
            << (long long)sol.sum_of_edge_costs_ << "\t"
            << (int32_t)((sol.path_.size() == 0) ? -1 : (int32_t)(sol.path_.size()-1)) << "\t"
            << parser->get_problemfile()
            << std::endl;
    }
}

void
run_cpdsearch(
        warthog::util::cfg* cfg,
        warthog::dimacs_parser* parser, 
        std::string alg_name)
{
    warthog::graph::xy_graph g;
    std::ifstream ifs;
    // We first load the xy_graph and its diff as we need them to be *read* in
    // reverse order.
    std::string xy_filename = cfg->get_param_value("input");
    if(xy_filename == "")
    {
        std::cerr << "parameter is missing: --input graph.xy [graph.xy.diff]"
                  << "[graph.xy.cpd]]\n";
        return;
    }

    ifs.open(xy_filename);
    if (!ifs.good())
    {
        std::cerr << "Could not open xy-graph: " << xy_filename << std::endl;
        return;
    }

    ifs >> g;
    ifs.close();

    // Check if we have a second parameter in the --input
    std::string diff_filename = cfg->get_param_value("input");
    if (diff_filename == "")
    {
        diff_filename = xy_filename + ".diff";
    }

    ifs.open(diff_filename);
    if (!ifs.good())
    {
        std::cerr <<
            "Could not open diff-graph: " << diff_filename << std::endl;
        return;
    }

    g.perturb(ifs);
    ifs.close();

    // read the cpd
    warthog::cpd::graph_oracle_base<warthog::cpd::FORWARD> oracle(&g);
    std::string cpd_filename = cfg->get_param_value("input");
    if(cpd_filename == "")
    {
        cpd_filename = xy_filename + ".cpd";
    }

    ifs.open(cpd_filename);
    if(ifs.is_open())
    {
        ifs >> oracle;
        ifs.close();
    }
    else
    {
        std::cerr << "Could not find CPD file '" << cpd_filename << "'\n";
        return;
    }

    warthog::simple_graph_expansion_policy expander(&g);
    warthog::cpd_heuristic_base<warthog::cpd::FORWARD> h(&oracle, 1.0);
    warthog::pqueue_min open;

    warthog::unidirectional_search<
        warthog::cpd_heuristic_base<warthog::cpd::FORWARD>,
        warthog::simple_graph_expansion_policy,
        warthog::pqueue_min,
        warthog::dummy_listener,
        warthog::admissibility_criteria::w_admissible, 
        warthog::feasibility_criteria::until_cutoff,
        warthog::reopen_policy::no>
            alg(&h, &expander, &open);

    // Set options for CPD searh
    std::stringstream ss;
    std::string scale = cfg->get_param_value("fscale");
    std::string tlim = cfg->get_param_value("uslim");
    warthog::search_parameters par;
    par.verbose_ = verbose;

    if (scale != "")
    {
        double f_scale;
        ss << scale;
        ss >> f_scale;

        if (f_scale > 0.0)
        {
            par.set_w_admissibility(f_scale);
        }
    }

    if (tlim != "")
    {
        uint32_t us_lim;
        ss << tlim;
        ss >> us_lim;

        if (us_lim > 0)
        {
            par.set_max_us_cutoff(us_lim);
        }
    }

    std::function<void(warthog::problem_instance*,
                         warthog::search_parameters*,
                         warthog::solution* sol)> alg_fn = 
        [&alg](auto *pi, auto *par, auto* sol) 
        {
            alg.get_path(pi, par, sol);
        };
    run_experiments(alg_fn, alg_name, &par, parser, std::cout);
}

void
run_astar(
        warthog::util::cfg* cfg,
        warthog::dimacs_parser* parser, 
        std::string alg_name)
{
    warthog::graph::xy_graph g;
    std::ifstream ifs;
    // We first load the xy_graph and its diff as we need them to be *read* in
    // reverse order.
    std::string xy_filename = cfg->get_param_value("input");
    if(xy_filename == "")
    {
        std::cerr << "parameter is missing: --input graph.xy [graph.xy.diff]"
                  << "[graph.xy.cpd]]\n";
        return;
    }

    ifs.open(xy_filename);
    if (!ifs.good())
    {
        std::cerr << "Could not open xy-graph: " << xy_filename << std::endl;
        return;
    }

    ifs >> g;
    ifs.close();

    warthog::simple_graph_expansion_policy expander(&g);
    warthog::euclidean_heuristic h(&g);
    warthog::pqueue_min open;

    warthog::unidirectional_search<
        warthog::euclidean_heuristic,
        warthog::simple_graph_expansion_policy,
        warthog::pqueue_min,
        warthog::dummy_listener,
        warthog::admissibility_criteria::w_admissible, 
        warthog::feasibility_criteria::until_cutoff,
        warthog::reopen_policy::no>
            alg(&h, &expander, &open);

    // set options
    std::stringstream ss;
    std::string scale = cfg->get_param_value("fscale");
    std::string tlim = cfg->get_param_value("uslim");
    warthog::search_parameters par;
    par.verbose_ = verbose;

    if (scale != "")
    {
        double f_scale;
        ss << scale;
        ss >> f_scale;

        if (f_scale > 0.0)
        {
            par.set_w_admissibility(f_scale);
        }
    }

    if (tlim != "")
    {
        uint32_t us_lim;
        ss << tlim;
        ss >> us_lim;

        if (us_lim > 0)
        {
            par.set_max_us_cutoff(us_lim);
        }
    }

    std::function<void(warthog::problem_instance*,
                         warthog::search_parameters*,
                         warthog::solution* sol)> alg_fn = 
        [&alg](auto *pi, auto *par, auto* sol) 
        {
            alg.get_path(pi, par, sol);
        };
    run_experiments(alg_fn, alg_name, &par, parser, std::cout);
}

int
main(int argc, char** argv)
{
    // parse arguments
    warthog::util::param valid_args[] =
    {
        {"alg",  required_argument, 0, 1},
        {"nruns",  required_argument, 0, 1},
        {"help", no_argument, &print_help, 1},
        {"checkopt",  no_argument, &checkopt, 1},
        {"verbose",  no_argument, &verbose, 1},
        {"noheader",  no_argument, &suppress_header, 1},
        {"input",  required_argument, 0, 1},
        {"problem",  required_argument, 0, 1},
        {"fscale", required_argument, 0, 1},
        {"uslim", required_argument, 0, 1},
        {"kmoves", required_argument, 0, 1},
        {0,  0, 0, 0}
    };

    warthog::util::cfg cfg;
    cfg.parse_args(argc, argv, "-f", valid_args);

    if(argc == 1 || print_help)
    {
        help();
        exit(0);
    }

    std::string alg_name = cfg.get_param_value("alg");
    std::string par_nruns = cfg.get_param_value("nruns");
    std::string problemfile = cfg.get_param_value("problem");

    if((alg_name == ""))
    {
        std::cerr << "parameter is missing: --alg\n";
        exit(0);
    }

    if((problemfile == ""))
    {
        std::cerr << "parameter is missing: --problem\n";
        exit(0);
    }

    if(par_nruns != "")
    {
       char* end;
       nruns = strtol(par_nruns.c_str(), &end, 10);
    }

    warthog::dimacs_parser parser;
    parser.load_instance(problemfile.c_str());
    if(parser.num_experiments() == 0)
    {
        std::cerr << "err; specified problem file contains no instances\n";
        exit(0);
    }

    if(alg_name == "cpdsearch")
    {
        run_cpdsearch(&cfg, &parser, alg_name);
    }
    else if(alg_name == "astar")
    {
        run_astar(&cfg, &parser, alg_name);
    }
    else
    {
        std::cerr << "invalid search algorithm\n";
    }
}
