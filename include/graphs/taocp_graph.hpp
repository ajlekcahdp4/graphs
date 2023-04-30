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

#include <algorithm>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <set>
#include <unordered_map>
#include <vector>

namespace graphs {

class punch_card_graph {
  using index_type = int;
  std::vector<int> data;
  std::unordered_map<int, index_type> val_to_idx_map;
  unsigned n_vertices{};
  unsigned mod{};

public:
  punch_card_graph(std::initializer_list<std::pair<int, int>> list) {
    std::set<int> vertices;
    auto n_edges = list.size();
    for (auto &&edge : list) {
      vertices.insert(edge.first);
      vertices.insert(edge.second);
    }
    n_vertices = vertices.size();
    mod = n_vertices + n_edges * 2;
    data.resize(4 * mod, 0);
    index_type i = 0;
    for (auto &&v : vertices) {
      val_to_idx_map[v] = i;
      data[i] = i;
      data[next(i)] = i;
      data[prev(i)] = i;
      ++i;
    }

    for (auto &&edge : list) {
      data[i] = i;
      data[i + 1] = i + 1;
      auto first_idx = val_to_idx_map.at(edge.first);
      auto second_idx = val_to_idx_map.at(edge.second);

      auto insert_edge = [this, &i](auto idx) {
        auto old_last = data[prev(idx)];
        auto old_next = data[next(old_last)];
        data[prev(idx)] = i;
        data[next(old_last)] = i;
        data[next(i)] = old_next;
        data[prev(i)] = old_last;
      };

      insert_edge(first_idx);
      data[from(i)] = edge.first;
      data[to(i)] = edge.second;
      ++i;
      insert_edge(second_idx);
      ++i;
    }
  }

  index_type from(index_type edge) const { return mod + edge; }

  index_type to(index_type edge) const {
    if (edge % 2) return mod + edge - 1;
    else return mod + edge + 1;
  }

  index_type next(index_type idx) { return mod * 2 + idx; }

  index_type prev(index_type idx) { return mod * 3 + idx; }

  std::ostream &dump(std::ostream &os) const {
    auto print = [this, &os](int begin, int end) {
      std::ios init(nullptr);
      init.copyfmt(os);

      for (auto i = begin; i < end; ++i)
        os << std::setw(3) << data[i] << " ";
      std::cout.copyfmt(init);
      os << "\n";
    };

    os << "a: ";
    print(0, mod);
    os << "t: ";
    print(mod, 2 * mod);
    os << "n: ";
    print(2 * mod, 3 * mod);
    os << "p: ";
    print(3 * mod, 4 * mod);
    return os;
  }
};
} // namespace graphs