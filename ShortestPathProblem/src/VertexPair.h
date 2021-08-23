#pragma once

// Struct to keep pairs of vertices
struct VertexPair
{
    size_t from;
    size_t to;

    bool operator==(const VertexPair& other) const
    {
        return (from == other.from && to == other.to);
    }
};

template <>
struct std::hash<VertexPair>
{
    std::size_t operator()(const VertexPair& v) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        // Compute individual hash values for first and
        // second and combine them using XOR and bit shifting:

        return hash<size_t>()(v.from) ^ (hash<size_t>()(v.to) << 1);
    }
};