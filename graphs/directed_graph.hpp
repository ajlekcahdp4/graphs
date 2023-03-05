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
#include <concepts>
#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace graphs {

template <typename T> class graph_node final : private std::vector<T> {
  T m_val;

  using vector = std::vector<T>;

public:
  using value_type = T;

  graph_node(const value_type val = value_type{}) : m_val{std::move(val)} {}

  operator value_type() { return m_val; }

  value_type value() const { return m_val; }

  bool add_adj(const value_type &val) {
    if (std::find(begin(), end(), val) != end()) return false;
    vector::push_back(val);
    return true;
  }

  using vector::begin;
  using vector::end;

  using vector::cbegin;
  using vector::cend;

  using vector::empty;
  using vector::size;
};

template <typename node_t>
  requires std::derived_from<node_t, graph_node<typename node_t::value_type>>
class directed_graph;

template <typename graph_t>
  requires std::derived_from<graph_t, directed_graph<typename graph_t::node_type>>
class breadth_first_traversal;

template <typename node_t>
  requires std::derived_from<node_t, graph_node<typename node_t::value_type>>
class directed_graph {
public:
  using size_type = std::size_t;
  using value_type = typename node_t::value_type;
  using node_type = node_t;

private:
  std::unordered_map<value_type, node_t> m_adj_list;
  size_type m_edge_n = 0;

public:
  directed_graph(){};

  virtual ~directed_graph() {}

  // inserts vertex
  virtual bool insert(const value_type &val) { return insert_base(val); }

  // inserts edge from first to second
  virtual bool insert(const value_type &first, const value_type &second) {
    if (!contains(first)) insert(first);
    if (!contains(second)) insert(second);
    return insert_base(first, second);
  }

  // checks if edge from first to second exists
  bool contains(const value_type &first, const value_type &second) const {
    if (!(m_adj_list.contains(first) && m_adj_list.contains(second))) return false;
    auto &&list = m_adj_list.at(first);
    if (std::find(list.begin(), list.end(), second) == list.end()) return false;
    return true;
  }

  // checks if vertex exists
  auto contains(const value_type &val) const { return m_adj_list.contains(val); }

  // returns number of edges in the graph
  size_type edges() const { return m_edge_n; }

  // returns number of vertices in the graph
  size_type vertices() const { return m_adj_list.size(); }

  // returns true if graph is empty
  bool empty() const { return !vertices(); }

  // Returns number of val's successors in the graph
  size_type successors(const value_type &val) const {
    if (!m_adj_list.contains(val)) throw std::logic_error{"Attempt to get number of successors of non-existent vertex"};
    return m_adj_list[val].size();
  }

  // returns true if first is directly connected to second
  bool connected(const value_type &first, const value_type &second) const {
    if (!(m_adj_list.contains(first) && m_adj_list.contains(second))) return false;
    auto &&first_node = m_adj_list.at(first);
    auto &&found = std::find(first_node.begin(), first_node.end(), second);
    if (found == first_node.end()) return false;
    return true;
  }

  // returns true if second is reachable from the first
  bool reachable(const value_type &first, const value_type &second) const {
    breadth_first_traversal search{*this};
    return search(first, [&second](auto &&val) { return val == second; });
  }

  auto find(const value_type &val) const { return m_adj_list.find(val); }

  auto begin() { return m_adj_list.begin(); }
  auto end() { return m_adj_list.end(); }
  auto begin() const { return m_adj_list.cbegin(); }
  auto end() const { return m_adj_list.cend(); }
  auto cbegin() const { return m_adj_list.cbegin(); }
  auto cend() const { return m_adj_list.cend(); }

protected:
  // insertes vertex if they are not already in the graph
  bool insert_base(const value_type &val) {
    auto &&[iter, inserted] = m_adj_list.insert({val, {val}});
    if (!inserted) return false;
    return true;
  }

  // inserts edge to the graph if it is not already inserted
  // inserts vertices if they are not already in the graph.
  bool insert_base(const value_type &vert1, const value_type &vert2) {
    if (!(m_adj_list.contains(vert1) && m_adj_list.contains(vert2))) return false;
    auto &&inserted = m_adj_list.at(vert1).add_adj(vert2);
    if (!inserted) return false;
    ++m_edge_n;
    return true;
  }
};

template <typename T> using basic_directed_graph = directed_graph<graph_node<T>>;

template <typename graph_t>
  requires std::derived_from<graph_t, directed_graph<typename graph_t::node_type>>
class breadth_first_traversal final {
  const graph_t &m_graph;
  using value_type = typename graph_t::value_type;

  enum class color_t {
    E_WHITE,
    E_GRAY,
    E_BLACK
  };

  struct bfs_node {
    unsigned m_dist = std::numeric_limits<unsigned>::max();
    color_t m_color = color_t::E_WHITE;
    bfs_node *m_prev = nullptr;
  };

public:
  breadth_first_traversal(const graph_t &graph) : m_graph{graph} {}

  // Breadth first traversal of the graph. Applying func to every vertex in order.
  // Returns true if predicate returns true at some vertex. Othervise returns false.
  template <typename F>
    requires std::is_same_v<std::invoke_result_t<F, value_type>, bool> bool
  operator()(const value_type &root_val, F func) const {
    if (!m_graph.contains(root_val)) throw std::logic_error{"Non-existing vertex root in BFS"};

    std::unordered_map<value_type, bfs_node> nodes;
    auto &&root_node = nodes.insert({root_val, {}}).first->second;
    root_node.m_color = color_t::E_GRAY;
    root_node.m_dist = 0;

    std::deque<value_type> que;
    que.push_back(root_val);
    while (!que.empty()) {
      auto &&curr = que.front();
      if (func(curr)) return true;
      que.pop_front();
      auto &&curr_node = nodes.insert({curr, {}}).first->second;
      auto &&curr_graph_node = m_graph.find(curr)->second;
      for (auto &&adj : curr_graph_node) {
        auto &&adj_node = nodes.insert({adj, {}}).first->second;
        if (adj_node.m_color == color_t::E_WHITE) {
          adj_node.m_color = color_t::E_GRAY;
          adj_node.m_dist = curr_node.m_dist + 1;
          adj_node.m_prev = &curr_node;
          que.push_back(adj);
        }
      }
      curr_node.m_color = color_t::E_BLACK;
    }
    return false;
  }

  // Breadth first traversal of the graph. Applying func to every vertex in order.
  template <typename F>
    requires std::is_same_v<std::invoke_result_t<F, value_type>, void>
  void operator()(const value_type &root_val, F func) const {
    if (!m_graph.contains(root_val)) throw std::logic_error{"Non-existing vertex root in BFS"};

    std::unordered_map<value_type, bfs_node> nodes;
    auto &&root_node = nodes.insert({root_val, {}}).first->second;
    root_node.m_color = color_t::E_GRAY;
    root_node.m_dist = 0;

    std::deque<value_type> que;
    que.push_back(root_val);
    while (!que.empty()) {
      auto &&curr = que.front();
      func(curr);
      que.pop_front();
      auto &&curr_node = nodes.insert({curr, {}}).first->second;
      auto &&curr_graph_node = m_graph.find(curr)->second;
      for (auto &&adj : curr_graph_node) {
        auto &&adj_node = nodes.insert({adj, {}}).first->second;
        if (adj_node.m_color == color_t::E_WHITE) {
          adj_node.m_color = color_t::E_GRAY;
          adj_node.m_dist = curr_node.m_dist + 1;
          adj_node.m_prev = &curr_node;
          que.push_back(adj);
        }
      }
      curr_node.m_color = color_t::E_BLACK;
    }
  }
};

template <typename graph_t>
  requires std::derived_from<graph_t, directed_graph<typename graph_t::node_type>>
std::vector<typename graph_t::value_type>
recursive_topo_sort(graph_t &graph, const typename graph_t::value_type &root_val) {
  using value_type = typename graph_t::value_type;
  enum class color_t {
    E_WHITE,
    E_GRAY,
    E_BLACK
  };

  struct bfs_node : public graph_t::node_type {
    int m_start = -1, m_finish = -1;
    color_t m_color = color_t::E_WHITE;
    bfs_node *m_prev = nullptr;

    bfs_node(const value_type &val) : graph_t::node_type{val} {}
  };

  int time = 0;
  std::vector<value_type> scheduled;
  std::unordered_map<value_type, bfs_node> nodes;

  for (auto &&val : graph)
    nodes.insert({val.first, val.first});

  const auto dfs_visit = [&time, &nodes, &graph, &scheduled](const value_type &val, auto &&dfs_visit) -> void {
    ++time;
    auto &&cur_node = nodes.at(val);
    cur_node.m_start = time;
    cur_node.m_color = color_t::E_GRAY;
    auto &&graph_node = graph.find(val)->second;
    for (auto &&adj : graph_node) {
      auto &&adj_node = nodes.at(adj);
      if (adj_node.m_color == color_t::E_WHITE) {
        adj_node.m_prev = &cur_node;
        dfs_visit(adj, dfs_visit);
      }
    }
    cur_node.m_color = color_t::E_BLACK;
    scheduled.push_back(val);
    ++time;
    cur_node.m_finish = time;
  };

  for (auto &&val : graph) {
    if (nodes.at(val.first).m_color == color_t::E_WHITE) dfs_visit(val.first, dfs_visit);
  }
  return std::vector(scheduled.rbegin(), scheduled.rend());
}

} // namespace graphs