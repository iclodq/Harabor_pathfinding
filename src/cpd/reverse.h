#ifndef __REVERSE_ORACLE_H_
#define __REVERSE_ORACLE_H_

#include "constants.h"
#include "cpd.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"

namespace warthog
{

namespace cpd
{

class reverse_oracle
{
    public:
        reverse_oracle(warthog::graph::xy_graph* g)
             : g_(g)
        {
            order_.resize(g_->get_num_nodes());
            fm_.resize(g_->get_num_nodes());
        }

        reverse_oracle() : g_(nullptr) { }

        virtual ~reverse_oracle() { }

        reverse_oracle(const reverse_oracle&) = default;

        bool
        operator==(const reverse_oracle& other)
        {
            if (order_ != other.order_)
            {
                return false;
            }

            if (fm_.size() != other.fm_.size())
            {
                return false;
            }

            for (size_t i = 0; i < fm_.size(); i++)
            {
                std::vector<warthog::cpd::rle_run32> row1 = fm_.at(i);
                std::vector<warthog::cpd::rle_run32> row2 = other.fm_.at(i);

                if (row1.size() != row2.size())
                {
                    return false;
                }

                for (size_t j = 0; j < row1.size(); j++)
                {
                    if (row1.at(j).data_ != row2.at(j).data_)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        inline uint32_t
        get_move(warthog::sn_id_t source_id,
                 warthog::sn_id_t target_id)
        {
            if(fm_.at(target_id).size() == 0) { return warthog::cpd::CPD_FM_NONE; }

            std::vector<warthog::cpd::rle_run32>& row = fm_.at(target_id);
            uint32_t source_index = order_.at(source_id);
            uint32_t end = (uint32_t)row.size();
            uint32_t begin = 0;
            while(begin<(end-1))
            {
                uint32_t mid = begin + ((end-begin)>>1);
                if(source_index < row.at(mid).get_index()) { end = mid ;  }
                else { begin = mid; }
            }
            return row.at(begin).get_move();
        }

        inline void
        clear()
        {
            order_.clear();
            fm_.clear();
        }

        inline void
        compute_dfs_preorder()
        {
            warthog::cpd::compute_dfs_preorder(g_, &order_);
        }

        // convert the column order into a map: from vertex id to its ordered
        // index
        inline void
        value_index_swap_array()
        {
            warthog::helpers::value_index_swap_array(order_);
        }

        // compress a given first-move table @param row and associate
        // the compressed result with source node @param source_id
        void
        add_row(uint32_t source_id,
                 std::vector<warthog::cpd::fm_coll>& row);

        inline warthog::graph::xy_graph*
        get_graph() { return g_; }

        inline void
        set_graph(warthog::graph::xy_graph* g)
        { g_ = g; }

        inline size_t
        mem()
        {
            size_t retval =
                g_->mem() +
                sizeof(uint32_t) * order_.size() +
                sizeof(std::vector<warthog::cpd::rle_run32>) * fm_.size();

            for(uint32_t i = 0; i < fm_.size(); i++)
            {
                retval += sizeof(warthog::cpd::rle_run32) * fm_.at(i).size();
            }

            return retval;
        }

        friend std::ostream&
        operator<<(std::ostream& out, warthog::cpd::reverse_oracle& o);

        friend std::istream&
        operator>>(std::istream& in, warthog::cpd::reverse_oracle& o);

        warthog::cpd::reverse_oracle&
        operator+=(const warthog::cpd::reverse_oracle &cpd);

        void
        compute_row(uint32_t source_id, warthog::search* dijk,
                    std::vector<warthog::cpd::fm_coll> &s_row);

    private:
        std::vector<std::vector<warthog::cpd::rle_run32>> fm_;
        std::vector<uint32_t> order_;
        warthog::graph::xy_graph* g_;
};

std::ostream&
operator<<(std::ostream& out, warthog::cpd::reverse_oracle& o);

std::istream&
operator>>(std::istream& in, warthog::cpd::reverse_oracle& o);

// helps to precompute first-move data
struct reverse_oracle_listener
{
    inline void
    generate_node(warthog::search_node *from, warthog::search_node *succ,
                  warthog::cost_t edge_cost, uint32_t edge_id)

    {
        if(from == nullptr) { return; } // start node has no predecessor

        double alt_g = from->get_g() + edge_cost;
        double g_val =
            succ->get_search_number() == from->get_search_number() ?
            succ->get_g() : DBL_MAX;
        // We record the optimal move towards a node which is the id of the
        // predecessor's edge
        graph::node* pred = oracle_->get_graph()->get_node(succ->get_id());
        graph::edge_iter eit = pred->find_edge(from->get_id());
        uint8_t fm = 1 << (eit - pred->outgoing_begin());

        assert(eit != pred->outgoing_end());

        //  update first move
        if(alt_g < g_val)
        {
            // Pointer arithmetic ftw
            s_row_->at(succ->get_id()) = fm;
            assert(s_row_->at(succ->get_id()));
        }

        // add to the list of optimal first moves
        if(alt_g == g_val)
        {
            s_row_->at(succ->get_id()) |= fm;
        }
    }

    inline void
    expand_node(warthog::search_node* current) { }

    inline void
    relax_node(warthog::search_node* current) { }

    warthog::cpd::reverse_oracle* oracle_;
    warthog::sn_id_t* source_id_; // Actually the target because we do a
                                  // bacwards search
    std::vector<warthog::cpd::fm_coll>* s_row_;

};

}

}

#endif // __REVERSE_H_
