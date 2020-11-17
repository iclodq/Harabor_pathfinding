/**
 * This file is used to create CPDs in an independent fashion.
 */
#include <iostream>
#include <fstream>
#include <getopt.h>
#include <omp.h>

#include "bidirectional_graph_expansion_policy.h"
#include "cfg.h"
#include "graph_oracle.h"
#include "log.h"
#include "reverse.h"
#include "xy_graph.h"

void
help()
{
  std::cerr << "Usage: ./make_cpd --input <graph.xy> "
            << "[--from/--to nid] [--output out] [--join list] "
            << "[--reverse] [--verbose] [--help]\n"
            << "\t--output    name of CPD file\n"
            << "\t--from/--to range of nodes to process for a partial CPD\n"
            << "\t--join      pass in a list of files to join partial CPDs\n"
            << "\t--reverse   create a reverse CPD\n"
            << "\t--verbose   enable progress output\n"
            << "\t--help      this message\n";
}

/**
 * Rebuild a CPD given a list of file containing its parts.
 *
 * The partial CPDs must be given in the order of the nodes.
 */
int
join_cpds(warthog::graph::xy_graph &g, warthog::cpd::graph_oracle &cpd,
          std::vector<std::string> file_list)
{
    for (auto name: file_list)
    {
        warthog::cpd::graph_oracle part(&g);
        std::ifstream ifs(name);

        if (!ifs.good())
        {
            std::cerr << "Cannot open file " << name << std::endl;
            return EXIT_FAILURE;
        }

        ifs >> part;
        ifs.close();

        cpd += part;
    }

    return 0;
}

template<class LISTENER>
void
compute_runs(std::vector<warthog::search*> &astar,
             std::vector<LISTENER*> &listeners,
             uint32_t node_count, int from, int to, bool verbose=false)
{
    if (to < 0) { to = node_count; }

    assert(to > 0);
    assert(from < to);
    assert((unsigned int)from < node_count);
    assert((unsigned int)to <= node_count);

    uint32_t node_end = to;
    unsigned char pct_done = 0;
    uint32_t nprocessed = 0;

    info(verbose, "Computing Dijkstra labels.");
    std::cerr << "progress: [";
    for(uint32_t i = 0; i < 100; i++) { std::cerr <<" "; }
    std::cerr << "]\rprogress: [";

    #pragma omp parallel
    {
        int thread_count = omp_get_num_threads();
        int thread_id = omp_get_thread_num();
        // thread-dependent structures
        warthog::search* dijk = astar.at(thread_id);
        LISTENER* listener = listeners.at(thread_id);
        std::vector<warthog::cpd::fm_coll> s_row(node_count);
        // warthog ids are 0-indexed, so no need to do anything.
        warthog::sn_id_t source_id = from + thread_id;
        listener->s_row_ = &s_row;

        while (source_id < node_end)
        {
            listener->source_id_ = &source_id;
            listener->oracle_->compute_row(source_id, dijk, s_row);

            // We increment the source by the number of threads to *jump* to
            // that id.
            source_id += thread_count;
            #pragma omp critical
            {
                nprocessed++;

                if ((nprocessed * 100 / (to - from)) > pct_done)
                {
                    std::cerr << "=";
                    pct_done++;
                }
            }
        }
    }

    std::cerr << std::endl;
}

int
make_rev_cpd(warthog::graph::xy_graph *g, int from, int to,
             std::string cpd_filename, bool verbose=false)
{
    size_t nthreads = omp_get_max_threads();
    std::vector<warthog::search*> astar(nthreads);
    std::vector<warthog::cpd::reverse_oracle_listener*> listeners(nthreads);
    warthog::cpd::reverse_oracle cpd(g);

    #pragma omp parallel for
    for (size_t t = 0; t < nthreads; t++)
    {
        // each thread has its own copy of Dijkstra and each
        // copy has a separate memory pool
        warthog::zero_heuristic* h = new warthog::zero_heuristic();
        warthog::pqueue_min* queue = new warthog::pqueue_min();
        warthog::bidirectional_graph_expansion_policy* expander =
            new warthog::bidirectional_graph_expansion_policy(g, true);
        warthog::cpd::reverse_oracle_listener* listener =
            new warthog::cpd::reverse_oracle_listener();
        warthog::search* dijk = new warthog::flexible_astar<
            warthog::zero_heuristic,
            warthog::bidirectional_graph_expansion_policy,
            warthog::pqueue_min,
            warthog::cpd::reverse_oracle_listener>(
            h, expander, queue, listener);

        listener->oracle_ = &cpd;

        astar.at(t) = dijk;
        listeners.at(t) = listener;
    }

    warthog::timer t;
    t.start();

    info(verbose, "Computing node ordering.");
    cpd.compute_dfs_preorder();

    compute_runs<warthog::cpd::reverse_oracle_listener>(
        astar, listeners, g->get_num_nodes(), from, to, verbose);

    // convert the column order into a map: from vertex id to its ordered index
    cpd.value_index_swap_array();

    t.stop();
    info(verbose, "total preproc time (seconds):", t.elapsed_time_sec());

    std::ofstream ofs(cpd_filename);

    if (!ofs.good())
    {
        std::cerr << "Could not open CPD file " << cpd_filename << std::endl;
        help();
        return EXIT_FAILURE;
    }

    info(verbose, "Writing results to", cpd_filename);
    ofs << cpd;
    ofs.close();

    return 0;
}

int
make_cpd(warthog::graph::xy_graph *g, int from, int to,
         std::string cpd_filename, bool verbose=false)
{
    size_t nthreads = omp_get_max_threads();
    std::vector<warthog::search*> astar(nthreads);
    std::vector<warthog::cpd::graph_oracle_listener *> listeners(nthreads);
    warthog::cpd::graph_oracle cpd(g);

    #pragma omp parallel for
    for (size_t t = 0; t < nthreads; t++)
    {
        // each thread has its own copy of Dijkstra and each
        // copy has a separate memory pool
        warthog::zero_heuristic* h = new warthog::zero_heuristic();
        warthog::pqueue_min* queue = new warthog::pqueue_min();
        warthog::bidirectional_graph_expansion_policy* expander =
            new warthog::bidirectional_graph_expansion_policy(g, false);
        warthog::cpd::graph_oracle_listener* listener =
            new warthog::cpd::graph_oracle_listener();
        warthog::search* dijk = new warthog::flexible_astar<
            warthog::zero_heuristic,
            warthog::bidirectional_graph_expansion_policy,
            warthog::pqueue_min, warthog::cpd::graph_oracle_listener>(
            h, expander, queue, listener);

        astar.at(t) = dijk;

        listener->oracle_ = &cpd;
        listeners.at(t) = listener;
    }

    warthog::timer t;
    t.start();

    info(verbose, "Computing node ordering.");
    cpd.compute_dfs_preorder();

    compute_runs<warthog::cpd::graph_oracle_listener>(
        astar, listeners, g->get_num_nodes(), from, to, verbose);

    // convert the column order into a map: from vertex id to its ordered index
    cpd.value_index_swap_array();

    t.stop();
    info(verbose, "total preproc time (seconds):", t.elapsed_time_sec());

    std::ofstream ofs(cpd_filename);

    if (!ofs.good())
    {
        std::cerr << "Could not open CPD file " << cpd_filename << std::endl;
        help();
        return EXIT_FAILURE;
    }

    info(verbose, "Writing results to", cpd_filename);
    ofs << cpd;
    ofs.close();

    return 0;
}

int
main(int argc, char *argv[])
{
    int verbose = 0;
    int reverse = 0;
    int view_help = 0;
    warthog::util::param valid_args[] =
    {
        {"from", required_argument, 0, 1},
        {"to", required_argument, 0, 1},
        {"input", required_argument, 0, 1},
        {"output", required_argument, 0, 1},
        {"join", required_argument, 0, 1},
        {"reverse", no_argument, &reverse, 1},
        {"verbose", no_argument, &verbose, 1},
        {"help", no_argument, &view_help, 1},
        {0, 0, 0, 0}
    };

    warthog::util::cfg cfg;
    cfg.parse_args(argc, argv, valid_args);

    if (view_help)
    {
        help();
        return EXIT_SUCCESS;
    }

    std::string s_from = cfg.get_param_value("from");
    std::string s_to = cfg.get_param_value("to");
    std::string xy_filename = cfg.get_param_value("input");
    std::string cpd_filename = cfg.get_param_value("output");

    if (xy_filename == "")
    {
        std::cerr << "Required argument --input [xy graph] missing."
                  << std::endl;
        help();
        return EXIT_FAILURE;
    }

    warthog::graph::xy_graph* g = new warthog::graph::xy_graph(0, "", true);
    std::ifstream ifs(xy_filename);

    if (!ifs.good())
    {
        std::cerr << "Cannot open file " << xy_filename << std::endl;
        help();
        return EXIT_FAILURE;
    }

    ifs >> *g;
    ifs.close();

    int from = 0;
    int to = -1;

    if (s_from != "")
    {
        from = std::stoi(s_from);

        if (from < 0)
        {
            std::cerr << "Argument --from [node id] cannot be negative."
                      << std::endl;
            help();
            return EXIT_FAILURE;
        }
    }

    if (s_to != "")
    {
        to = std::stoi(s_to);
    }

    int status = 0;

    // No error, proceed
    if (cpd_filename == "")
    {
        // Use default name
        cpd_filename = xy_filename + ".cpd";
    }

    if (cfg.get_num_values("join") > 0)
    {
        std::vector<std::string> names;
        std::string part;
        // TODO
        warthog::cpd::graph_oracle cpd(g);

        while (true)
        {
            part = cfg.get_param_value("join");

            if (part == "") { break; }
            names.push_back(part);
        }
        cpd.clear();
        status = join_cpds(*g, cpd, names);
    }
    else
    {
        if (reverse)
        {
            status = make_rev_cpd(g, from, to, cpd_filename, verbose);
        }
        else
        {
            status = make_cpd(g, from, to, cpd_filename, verbose);
        }
    }

    return status;
}
