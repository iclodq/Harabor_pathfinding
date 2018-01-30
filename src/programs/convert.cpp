#include "bbaf_labelling.h"
#include "cfg.h"
#include "contraction.h"

int
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"input",  required_argument, 0, 1},
	};

    warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "-f", valid_args);
     
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || orderfile == "" ||
             arclabels_file == "" || partfile == "")
    {
        std::cerr << "err; insufficient input parameters. "
                  << " required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return EINVAL;
    }

    // load up the graph
    warthog::graph::planar_graph g;
    if(!g.load_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return EINVAL;
    }

    // load up the partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part))
    {
        std::cerr << "err; could not load partition file\n"; 
        return EINVAL;
    }

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return EINVAL;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr << "contraction order length not equal to "
            << " number of graph nodes\n";
        return EINVAL;
    }

    // load up the arc-flags
    std::shared_ptr<warthog::label::bbaf_labelling> lab(
            warthog::label::bbaf_labelling::load(
                arclabels_file.c_str(), &g, &part));


    // converting
    std::cerr << "converting ... ";
    std::unordered_map<uint32_t, uint32_t> index2id_before;
    std::unordered_map<uint32_t, uint32_t> id2index_before;
    uint32_t count = 0;
    for(uint32_t n_id = 0; n_id < g.get_num_nodes(); n_id++)
    {
        count++;
        index2id_before.clear();
        id2index_before.clear();

        std::vector<std::vector<warthog::label::bbaf_label>>* raw = 
            lab->get_raw_labels();

        warthog::graph::node* n = g.get_node(n_id);
        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge_iter it = (n->outgoing_begin() + i);
            id2index_before[it->node_id_] = i;
            index2id_before[i] = it->node_id_;
//            std::cerr << "\nindex " << i << " head " << it->node_id_ << "\n";
//            lab->print_label(std::cerr, lab->get_label(n_id, i));
        }

        // sort the edges of every node (all up succ first, then all down):
        std::sort(n->outgoing_begin(), n->outgoing_end(), 
                [&order] 
                (warthog::graph::edge& first,
                 warthog::graph::edge& second) -> bool
                {
                    uint32_t f_rank = order.at(first.node_id_);
                    uint32_t s_rank = order.at(second.node_id_);
                    return f_rank > s_rank;
                } );

        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge_iter it = (n->outgoing_begin() + i);
            uint32_t old_index_for_edge = id2index_before[it->node_id_];
            uint32_t old_edge_at_index_i = index2id_before[i];

            warthog::label::bbaf_label tmp = raw->at(n_id).at(i);
            raw->at(n_id).at(i) = raw->at(n_id).at(old_index_for_edge); 
            raw->at(n_id).at(old_index_for_edge) = tmp;
            id2index_before[it->node_id_] = i;
            index2id_before[i] = it->node_id_;
            index2id_before[old_index_for_edge] = old_edge_at_index_i;
            id2index_before[old_edge_at_index_i] = old_index_for_edge;
        }

//        std::cerr << "\nafter conversion\n";
        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge_iter it = (n->outgoing_begin() + i);
            id2index_before[it->node_id_] = i;
            index2id_before[i] = it->node_id_;
//            std::cerr << "\nindex " << i << " head " << it->node_id_ << "\n";
//            lab->print_label(std::cerr, lab->get_label(n_id, i));
        }

        // sanity
        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge_iter it = (n->outgoing_begin() + i);
            assert(id2index_before[it->node_id_] == i);
            assert(index2id_before[i] == it->node_id_);
        }
//        return 0;
    }
    std::cerr << count << " nodes. all done!\n";
    lab->print(std::cout);
    return 0;
}
