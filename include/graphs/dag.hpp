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

#include <cassert>

namespace graphs {

template <typename T, std::invocable<T> hash_t = std::hash<T>> struct dag : public basic_directed_graph<T, void, void> {
  using base = basic_directed_graph<T, void, void>;
  using base::contains;
  using base::insert;
  using typename base::value_type;

  // // inserts edge from first to second to the DAG. If NDEBUG not defined checks if cycle is possible.
  // bool insert(const value_type &first, const value_type &second) override {
  //   if (!contains(first)) insert(first);
  //   if (!contains(second)) insert(second);
  //   assert(!base::reachable(second, first) && "Attempt to create cycle in DAG");
  //   return base::insert_base(first, second);
  // }
};

} // namespace graphs