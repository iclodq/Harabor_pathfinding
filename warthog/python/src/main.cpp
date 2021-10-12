/**
 * What we are trying to do with this module is to have two ways of calling
 * warthog's CPD search:
 *
 *  1. Load objects into Python and call the search.
 *
 *  2. Pass raw data and build objects on the fly before calling the search.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <fstream>
#include <sstream>

#include "cpd_heuristic.h"
#include "cpd_search.h"
#include "graph.h"
#include "graph_expansion_policy.h"
#include "graph_oracle.h"
#include "log.h"
#include "timer.h"
#include "xy_graph.h"

namespace py = pybind11;
using cpd_search_t = warthog::cpd_search<warthog::cpd_heuristic, warthog::simple_graph_expansion_policy>;

void
open_file(std::ifstream& ifs, std::string& filename)
{
    ifs.open(filename);

    if(!ifs.good())
    {
        throw std::ios_base::failure("Could not open " + filename);
    }
}

PYBIND11_MODULE(pyhog, m)
{
    // Opaque objects, declared so that Python does not crash but should not be
    // interacted with.
    py::class_<warthog::graph::node>(m, "node")
        .def("__repr__", [](warthog::graph::node& n)
        {
            std::ostringstream os;
            os << "<pyhog.node deg=" << n.out_degree() << ">";

            return os.str();
        });

    py::class_<warthog::graph::edge>(m, "edge")
        .def("__repr__", [](warthog::graph::edge& e)
        {
            std::ostringstream os;
            os << "<pyhog.";
            e.print(os);
            os << ">";

            return os.str();
        })
        .def_readonly("node_id", &warthog::graph::edge::node_id_)
        .def_readonly("wt", &warthog::graph::edge::wt_);

    // Class object
    py::class_<warthog::graph::xy_graph>(m, "xy_graph")
        .def(py::init<>(), "Graph class to use with warthog.")
        .def("get_num_nodes", &warthog::graph::xy_graph::get_num_nodes)
        .def("get_num_edges", &warthog::graph::xy_graph::get_num_edges_out)
        .def(
            "get_node",
            &warthog::graph::xy_graph::get_node,
            "`pyhog.node` is opaque, cannot interact with the results")
        .def(
            "get_edge",
            [](warthog::graph::xy_graph& g, uint32_t head, uint32_t tail)
            {
                warthog::graph::node* n = g.get_node(head);
                if(n == nullptr)
                {
                    std::stringstream ss;
                    ss << "Node not found in" << __func__ << "(" << head << ", "
                       << tail << ")";
                    throw std::range_error(ss.str());
                }

                warthog::graph::edge_iter e = n->find_edge(tail);
                if(e == n->outgoing_end())
                {
                    std::stringstream ss;
                    ss << "Edge not found in" << __func__ << "(" << head << ", "
                       << tail << ")";
                    throw std::range_error(ss.str());
                }

                return *e;
            },
            "`pyhog.edge` is opaque, cannot interact with the results")
        .def(
            "__repr__",
            [](warthog::graph::xy_graph& xy)
            {
                std::ostringstream os;
                os << "<pyhog.xy n=" << xy.get_num_nodes() << ", "
                   << "e=" << xy.get_num_edges_out() << ">";

                return os.str();
            })
        .def(
            "__eq__",
            [](warthog::graph::xy_graph& g, warthog::graph::xy_graph& xy)
            {
                return xy == g;
            })
        .def(
            "load", [](warthog::graph::xy_graph& g, std::string& xy_filename)
            {
                std::ifstream ifs;

                g.set_filename(xy_filename.c_str());

                open_file(ifs, xy_filename);
                ifs >> g;
                ifs.close();
            },
            "Load an existing xy-graph file.")
        .def(
            "perturb",
            [](warthog::graph::xy_graph& g, std::string& diff_filename)
            {
                std::ifstream ifs;

                open_file(ifs, diff_filename);
                g.perturb(ifs);
                ifs.close();

            },
            "Perturg the current graph given another xy-graph file.")
        .def(
            "perturb",
            [](warthog::graph::xy_graph& g,
               std::vector<std::tuple<int, int, double>>& modif)
            {
                std::vector<std::pair<uint32_t, warthog::graph::edge>> edges;
                for(auto& e : modif)
                {
                    uint32_t head = std::get<0>(e);
                    uint32_t tail = std::get<1>(e);
                    warthog::graph::edge_cost_t weight = std::get<2>(e);
                    edges.push_back({head, warthog::graph::edge(tail, weight)});
                }

                g.perturb(edges);
            },
            "Perturb the current graph given a list of (head, tail, cost).")
        .def(
            "add_node",
            static_cast<
            uint32_t (warthog::graph::xy_graph::*)(int32_t, int32_t)>(
                &warthog::graph::xy_graph::add_node),
            "Add a node to the current graph with coordinates (x, y).")
        .def(
            "add_edge",
            [](warthog::graph::xy_graph& g, uint32_t head, uint32_t tail,
               warthog::graph::edge_cost_t weight)
            {
                warthog::graph::node* n = g.get_node(head);
                if(n == nullptr)
                {
                    std::stringstream ss;
                    ss << "Node not found in" << __func__ << "(" << head << ", "
                       << tail << ")";
                    throw std::range_error(ss.str());
                }

                n->add_outgoing(warthog::graph::edge(tail, weight));

                return n;
            },
            "As `pyhog.node` is opaque, use this method to add edges to a node "
            "in the current graph.");

    py::class_<warthog::cpd::graph_oracle>(m, "graph_oracle")
        .def(
            py::init<warthog::graph::xy_graph*>(),
            "A graph oracle is based on an xy-graph and gives search directions.")
        .def(
            "get_num_nodes",
            [](warthog::cpd::graph_oracle& cpd)
            {
                return cpd.get_graph()->get_num_nodes();
            })
        .def("set_graph", &warthog::cpd::graph_oracle::set_graph)
        .def(
            "load",
            [](warthog::cpd::graph_oracle& cpd, std::string& cpd_filename)
            {
                std::ifstream ifs;

                open_file(ifs, cpd_filename);
                ifs >> cpd;
                ifs.close();
            },
            "Load CPD from file.")
        .def(
            "__eq__",
            [](warthog::cpd::graph_oracle& cpd,
               warthog::cpd::graph_oracle& oracle) { return cpd == oracle; })
        .def("__repr__", [](warthog::cpd::graph_oracle& cpd)
        {
            std::ostringstream os;
            os << "<pyhog.cpd n=" << cpd.get_graph()->get_num_nodes() << ">";

            return os.str();
        });

    py::class_<warthog::cpd_heuristic>(m, "cpd_heuristic")
        .def(py::init<warthog::cpd::graph_oracle*>());

    py::class_<warthog::pqueue_min>(m, "pqueue")
        .def(py::init<>());

    py::class_<warthog::simple_graph_expansion_policy>(m, "graph_expander")
        .def(py::init<warthog::graph::xy_graph*>());

    py::class_<warthog::solution>(m, "solution")
        .def(
            py::init<>(),
            "Solution object contains statistics, and the cost and path found.")
        .def_readonly("sum_of_edge_costs", &warthog::solution::sum_of_edge_costs_)
        .def_readonly("time_elapsed_nano", &warthog::solution::time_elapsed_nano_)
        .def_readonly("nodes_expanded", &warthog::solution::nodes_expanded_)
        .def_readonly("nodes_inserted", &warthog::solution::nodes_inserted_)
        .def_readonly("nodes_updated", &warthog::solution::nodes_updated_)
        .def_readonly("nodes_touched", &warthog::solution::nodes_touched_)
        .def_readonly("nodes_surplus", &warthog::solution::nodes_surplus_)
        .def_readonly("path", &warthog::solution::path_)
        .def("__repr__", [](const warthog::solution& sol)
        {
            std::stringstream ss;
            sol.print(ss);
            return "<pyhog.solution " + ss.str() + ">";
        });

    py::class_<warthog::problem_instance>(m, "pi")
        .def(py::init<>(), "A problem instance represents an (o, d)-pair")
        .def(py::init<warthog::sn_id_t, warthog::sn_id_t>())
        .def_readwrite("start_id", &warthog::problem_instance::start_id_)
        .def_readwrite("target_id", &warthog::problem_instance::target_id_)
        .def_readonly("instance_id", &warthog::problem_instance::instance_id_)
        .def_readwrite("verbose", &warthog::problem_instance::verbose_);

    py::class_<cpd_search_t>(m, "cpd_search")
        .def(py::init<
             warthog::cpd_heuristic*, warthog::simple_graph_expansion_policy*,
             warthog::pqueue_min*>(),
            "CPD Search algorithm, needs a heuristic, an exander and a queue.")
        .def(
            "get_path",
            &cpd_search_t::get_path,
            "Default interface: execute search and return the path found.")
        .def(
            "get_pathcost",
            &cpd_search_t::get_pathcost,
            "Default interface: execute search and return the cost of the path "
            "found.")
        .def(
            "path",
            [](cpd_search_t& s, warthog::sn_id_t source, warthog::sn_id_t target)
            {
                warthog::problem_instance pi(source, target);
                warthog::solution sol;

                s.get_path(pi, sol);

                return sol.path_;
            },
            "Direct interface: execute search between @param source and @param "
            "target and return the path found.")
        .def(
            "cost",
            [](cpd_search_t& s, warthog::sn_id_t source, warthog::sn_id_t target)
            {
                warthog::problem_instance pi(source, target);
                warthog::solution sol;

                s.get_path(pi, sol);

                return sol.sum_of_edge_costs_;
            },
            "Direct interface: execute search between @param source and @param "
            "target and return the cost of the path found.")
        .def(
            "solution",
            [](cpd_search_t& s, warthog::sn_id_t source, warthog::sn_id_t target)
            {
                warthog::problem_instance pi(source, target);
                warthog::solution sol;

                s.get_path(pi, sol);

                return sol;
            },
            "Direct interface: execute search between @param source and @param "
            "target and return the solution object.");

    m.def("create_cpd_search", [](
              warthog::graph::xy_graph* g, warthog::cpd::graph_oracle* oracle)
    {
        // TODO Mem leak?
        auto* expander = new warthog::simple_graph_expansion_policy(g);
        auto* h = new warthog::cpd_heuristic(oracle, 1.0);
        auto* open = new warthog::pqueue_min();

        return new warthog::cpd_search<
            warthog::cpd_heuristic,
            warthog::simple_graph_expansion_policy,
            warthog::pqueue_min>(h, expander, open);
    }, "Create a CPD Search using an xy-graph and a CPD oracle.");

    m.def("create_cpd_search", [](std::string& xy_name)
    {
        std::ifstream ifs;
        auto* g = new warthog::graph::xy_graph();
        open_file(ifs, xy_name);
        ifs >> *g;
        ifs.close();

        auto* oracle = new warthog::cpd::graph_oracle(g);
        std::string cpd_filename = xy_name + ".cpd";
        open_file(ifs, cpd_filename);
        ifs >> *oracle;
        ifs.close();

        std::string diff_filename = xy_name + ".diff";
        open_file(ifs, diff_filename);
        g->perturb(ifs);
        ifs.close();

        auto* expander = new warthog::simple_graph_expansion_policy(g);
        auto* h = new warthog::cpd_heuristic(oracle, 1.0);
        auto* open = new warthog::pqueue_min();

        return new warthog::cpd_search<
            warthog::cpd_heuristic,
            warthog::simple_graph_expansion_policy,
            warthog::pqueue_min>(h, expander, open);
    }, "Create a CPD Search from three files using @param xy_name: <xy_name> "
        "<xy_name.cpd> <xy_name.diff>");
}
