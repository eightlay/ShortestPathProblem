#pragma once

// Struct to keep pairs of vertices
struct Arc
{
    Arc(size_t from_, size_t to_) : from(from_), to(to_) {}
    size_t from;
    size_t to;
};
