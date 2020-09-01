//
// Run Frank-Wolfe on a given instance.
//

#include "forward.h"
#include "getopt.h"
#include "cfg.h"
#include "graph_oracle.h"
#include <cstdlib>

// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;

/**
 * Handle CLI arguments.
 */
int
run_frank_wolfe(std::string xy_filename, std::string problemfile)
{
    warthog::graph::xy_graph g;
    std::ifstream ifs(xy_filename);
    ifs >> g;
    ifs.close();

    warthog::dimacs_parser parser;
    parser.load_instance(problemfile.c_str());
    if(parser.num_experiments() == 0)
    {
        std::cerr << "err; specified problem file contains no instances\n";
        return EXIT_FAILURE;
    }

    warthog::simple_graph_expansion_policy expander(&g);
    warthog::zero_heuristic h;
    warthog::pqueue<warthog::cmp_less_search_node_f_only, warthog::min_q> open;

    warthog::flexible_astar<
        warthog::zero_heuristic,
        warthog::simple_graph_expansion_policy,
        warthog::pqueue<warthog::cmp_less_search_node_f_only, warthog::min_q>>
            alg(&h, &expander, &open);

    for(auto it = parser.experiments_begin();
            it != parser.experiments_end();
            it++)
    {
        warthog::dimacs_parser::experiment exp = (*it);
        warthog::solution sol;
        warthog::sn_id_t start_id = exp.source;
        warthog::sn_id_t target_id = exp.p2p ? exp.target : warthog::SN_ID_MAX;
        warthog::problem_instance pi(start_id, target_id, verbose);

        // TODO Here you have a Dijkstra algorithm ready. You can run it by
        // TODO calling:
        alg.get_path(pi, sol);
        // TODO Experiments data will be save in @sol@.
        std::cout << sol << std::endl;
    }

    return EXIT_SUCCESS;
}

int
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] =
	{
		// {"alg",  required_argument, 0, 1},
		// {"nruns",  required_argument, 0, 1},
		{"help", no_argument, &print_help, 1},
		{"verbose",  no_argument, &verbose, 1},
		{"input",  required_argument, 0, 1},
		{"problem",  required_argument, 0, 1},
		{0,  0, 0, 0}
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "-f", valid_args);

    if(argc == 1 || print_help)
    {
        std::cerr
            << "Program to compute user equilibrium with Frank-Wolfe" << std::endl
            << std::endl
            << "\t--input\tInput graph (xy-graph)" << std::endl
            << "\t--problem\tInstance file (o/d pairs)" << std::endl
            << "\t--verbose\tprint debug info" << std::endl;
        return EXIT_FAILURE;
    }

    std::string problemfile = cfg.get_param_value("problem");

    if((problemfile == ""))
    {
        std::cerr << "parameter is missing: --problem\n";
        return EXIT_FAILURE;
    }

    std::string xy_filename = cfg.get_param_value("input");
    if(xy_filename == "")
    {
        std::cerr << "parameter is missing: --input [xy-graph file]\n";
        return EXIT_FAILURE;
    }

    return run_frank_wolfe(xy_filename, problemfile);
}
