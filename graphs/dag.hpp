/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <tsimmerman.ss@phystech.edu>, <alex.rom23@mail.ru> wrote this file.  As long
 * as you retain this notice you can do whatever you want with this stuff. If we
 * meet some day, and you think this stuff is worth it, you can buy me a beer in
 * return.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "directed_graph.hpp"

namespace graphs {

template <typename T> struct dag : public directed_graph<graph_node<T>> {
  using base = directed_graph<graph_node<T>>;
  using base::contains;
  using base::insert;
  using typename base::value_type;
  bool insert(const value_type &first, const value_type &second) override {
    if (!contains(first)) insert(first);
    if (!contains(second)) insert(second);
    breadth_first search{*this};
    auto &&cycle_possible = search(second, [&first](auto &&val) { return val == first; });
    if (cycle_possible) return false;
    return base::insert_base(first, second);
  }
};

} // namespace graphs