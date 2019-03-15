#ifndef WARTHOG_NODE_ID_H
#define WARTHOG_NODE_ID_H

namespace warthog
{

union packed_time_and_id
{
    uint64_t t_id : 64;
    struct
    {
        uint32_t id : 32;
        int32_t t : 32;
    };

    packed_time_and_id() noexcept = default;
    explicit packed_time_and_id(uint64_t t_id) noexcept : t_id(t_id) {}
    explicit packed_time_and_id(int32_t t, uint32_t id) noexcept : id(id), t(t) {}
};

inline bool operator==(const packed_time_and_id lhs, const packed_time_and_id rhs)
{
    return lhs.t_id == rhs.t_id;
}

static_assert(std::is_trivially_copyable<packed_time_and_id>::value);

}

#endif
