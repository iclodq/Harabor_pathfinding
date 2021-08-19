// -*- compile-command: "cd .. && make debug test/distance" -*-
#include "cfg.h"
#include "log.h"
#include "xy_graph.h"
#include "geography.h"
#include "euclidean_heuristic.h"
#include "haversine_heuristic.h"

#include <limits>
#include <functional>

int suppress_header = 0;

using namespace warthog::geo;
using dist_fn = std::function<double(double, double, double, double)>;

std::array<std::string, 9> alg_names {
    "spherical", "great-circle", "vincenty", "vincenty-exact", "haversine",
    "haversine-direct", "fast-haversine", "haversine-approx", "euclidean"};

std::array<dist_fn, 9> algs {
    spherical_distance,
    great_circle_distance,
    vincenty_distance,
    exact_distance,
    [](double xs, double ys, double xt, double yt) -> double
    {
        return warthog::haversine_heuristic::h(xs, ys, xt, yt);
    },
    haversine,
    fast_haversine,
    haversine_approx,
    [](double xs, double ys, double xt, double yt) -> double
    {
        return warthog::euclidean_heuristic::h(xs, ys, xt, yt);
    }
};

std::vector<double>
lat_lon(warthog::graph::xy_graph *g)
{
    size_t num_nodes = g->get_num_nodes();
    std::vector<double> coords(num_nodes * 2);

    for (size_t i = 0; i < num_nodes; i++) {
        int32_t x;
        int32_t y;

        g->get_xy(i, x, y);
        coords.at(i * 2) = x / warthog::DIMACS_RATIO;
        coords.at(i * 2 + 1) = y / warthog::DIMACS_RATIO;
    }

    return coords;
}

void
run_precision(warthog::dimacs_parser& parser)
{
    std::cout.precision(std::numeric_limits<double>::max_digits10);

    for(auto e = parser.edges_begin(); e != parser.edges_end(); e++)
    {
        warthog::dimacs_parser::node head = parser.get_dimacs_node(e->head_id_);
        warthog::dimacs_parser::node tail = parser.get_dimacs_node(e->tail_id_);

        double lon_a = tail.x_ / warthog::DIMACS_RATIO;
        double lat_a = tail.y_ / warthog::DIMACS_RATIO;
        double lon_b = head.x_ / warthog::DIMACS_RATIO;
        double lat_b = head.y_ / warthog::DIMACS_RATIO;

        std::cout << lat_a << "," << lon_a << "," << lat_b << "," << lon_b
            << "," << e->weight_;

        for(size_t alg = 0; alg < alg_names.size(); alg++)
        {
            std::cout << "," << algs.at(alg)(lon_a, lat_a, lon_b, lat_b);
        }
        std::cout << std::endl;
    }
}

void
run_distance(size_t alg, std::vector<double>& coords, size_t nruns)
{
    assert(coords.size() % 2 == 0);

    dist_fn dist = algs.at(alg);
    warthog::timer t;
    size_t i = 0;

    t.start();
    while(nruns > 0)
    {
        double x = coords.at(i);
        double y = coords.at(i + 1);
        i++;

        for(size_t j = 0; j < coords.size(); j += 2)
        {
            if(i == j) { continue; }

            dist(x, y, coords.at(j), coords.at(j + 1));
            nruns--;

            if(nruns <= 0) { break; }
        }
    }

    t.stop();

    std::cout << alg_names.at(alg) << "," << coords.size() / 2 << ","
              << t.elapsed_time_nano() << std::endl;
}

int
main(int argc, char** argv)
{
    // parse arguments
    warthog::util::param valid_args[] =
    {
        {"alg",  required_argument, nullptr, 1},
        {"nruns",  required_argument, nullptr, 1},
        // {"help", no_argument, &print_help, 1},
        // {"checkopt",  no_argument, &checkopt, 1},
        // {"verbose",  no_argument, &verbose, 1},
        {"noheader",  no_argument, &suppress_header, 1},
        {"input",  required_argument, nullptr, 1},
        {"dist", required_argument, nullptr, 1},
        // {"problem",  required_argument, 0, 1},
        // {"fscale", required_argument, 0, 1},
        // {"uslim", required_argument, 0, 1},
        // {"kmoves", required_argument, 0, 1},
        {nullptr,  0, nullptr, 0}
    };

    warthog::util::cfg cfg;
    cfg.parse_args(argc, argv, valid_args);

    size_t nargs = cfg.get_num_values("input");
    switch(nargs)
    {
    case 0:
        std::cerr << "Need an --input [<xy-graph>|<co-file> <gr-file>]";
        return EXIT_FAILURE;

    case 1:
    {
        std::string xy_filename = cfg.get_param_value("input");
        if (xy_filename.empty())
        {
            std::cerr << "Need an --input <xy-graph>" << std::endl;
            return EXIT_FAILURE;
        }

        warthog::graph::xy_graph g;
        std::ifstream ifs(xy_filename);
        ifs >> g;

        std::vector<double> coords = lat_lon(&g);

        size_t nruns = coords.size() / 2;
        std::string s_runs = cfg.get_param_value("nruns");
        if(!s_runs.empty())
        {
            nruns = std::stoi(s_runs);
            assert(nruns > 0);
        }

        if(suppress_header != 1)
        {
            std::cout << "alg,size,nanos" << std::endl;
        }

        std::string which = cfg.get_param_value("alg");
        if(which.empty())           // run all algos
        {
            for(size_t alg = 0; alg < alg_names.size(); alg++)
            {
                run_distance(alg, coords, nruns);
            }
        }
        else
        {
            auto alg = std::find(alg_names.begin(), alg_names.end(), which);
            if(alg > alg_names.end())
            {
                std::cerr << "Algorithm must be one of ";
                for(auto& a : alg_names)
                {
                    std::cerr << a << ", ";
                }
                std::cerr << "or not provided; not: " << which << std::endl;
                return EXIT_FAILURE;
            }

            run_distance(alg - alg_names.begin(), coords, nruns);
        }

        break;
    }
    case 2:
    {
        std::string co_file = cfg.get_param_value("input");
        std::string gr_file = cfg.get_param_value("input");
        warthog::dimacs_parser dist(co_file.c_str(), gr_file.c_str());

        if(dist.get_num_edges() == 0)
        {
            std::cerr << "Instance contains no entries.";
            return EXIT_FAILURE;
        }
       
        if(suppress_header != 1)
        {
            std::cout << "lat_a,lon_a,lat_b,lon_b,base";
            for(auto& a : alg_names)
            {
                std::cout << "," << a;
            }
            std::cout << std::endl;
        }

        run_precision(dist);
        break;
    }
    default:
        std::cerr
            << "Too many arguments (" << nargs
            << ") to --input [<xy-file>|<co-file> <gr-file>]" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
