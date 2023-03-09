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
#include <utility>
#include <vector>

namespace graphs {

namespace detail {

// clang-format off
template <typename t_comp, typename t_key>
concept comparator = requires(t_comp functor, t_key a, t_key b) {
  { functor(a, b) } -> std::convertible_to<bool>;
};
// clang-format on

template <typename t_hash, typename t_key>
concept hasher = std::invocable<t_hash, t_key>;

template <typename t_key, typename t_attr, typename t_edge> struct adjacency_list_traits {
  using key_type = t_key;
  using attr_type = t_attr;
  using edge_type = t_edge;

  using value_type = std::pair<key_type, attr_type>;
  using entry_type = std::pair<value_type, edge_type>;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key(value_type &val) { return val.first; }
  static const key_type &get_key(const value_type &val) { return val.first; }
};

template <typename t_key, typename t_attr> struct adjacency_list_traits<t_key, t_attr, void> {
  using key_type = t_key;
  using attr_type = t_attr;

  using value_type = std::pair<key_type, attr_type>;
  using entry_type = value_type;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key(value_type &val) { return val.first; }
  static const key_type &get_key(const value_type &val) { return val.first; }
};

template <typename t_key, typename t_edge> struct adjacency_list_traits<t_key, void, t_edge> {
  using key_type = t_key;
  using edge_type = t_edge;

  using value_type = key_type;
  using entry_type = std::pair<value_type, edge_type>;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key(value_type &val) { return val; }
  static const key_type &get_key(const value_type &val) { return val; }
};

template <typename t_key> struct adjacency_list_traits<t_key, void, void> {
  using key_type = t_key;

  using value_type = key_type;
  using entry_type = key_type;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key(value_type &val) { return val; }
  static const key_type &get_key(const value_type &val) { return val; }
};

template <typename t_key, typename t_attr, typename t_edge>
using adjacency_list_t = typename adjacency_list_traits<t_key, t_attr, t_edge>::type;

template <typename t_key, typename t_attr, typename t_edge>
class basic_graph_node : private adjacency_list_t<t_key, t_attr, t_edge> {
private:
  using traits = adjacency_list_traits<t_key, t_attr, t_edge>;

public:
  using value_type = typename traits::value_type;
  using entry_type = typename traits::entry_type;

  using attr_type = t_attr;
  using edge_type = t_edge;
  using key_type = t_key;

private:
  value_type m_val;

  using adjacency_list = typename traits::type;

public:
  explicit basic_graph_node(const value_type &val) : m_val{val} {}
  explicit basic_graph_node(value_type &&val) : m_val{std::move(val)} {}

  using adjacency_list::begin;
  using adjacency_list::cbegin;
  using adjacency_list::cend;
  using adjacency_list::empty;
  using adjacency_list::end;
  using adjacency_list::size;

  const key_type &key() const & { return traits::get_key(m_val); }
  const value_type &value() const & { return m_val; }

  bool add_adj(const entry_type &val) {
    if (std::find(begin(), end(), val) != end()) return false;
    adjacency_list::push_back(val);
    return true;
  }

  bool add_adj(entry_type &&val) {
    if (std::find(begin(), end(), val) != end()) return false;
    adjacency_list::push_back(std::move(val));
    return true;
  }
};

} // namespace detail

// clang-format off
template <typename t_node>
concept graph_node = requires() {
  typename t_node::key_type;
  typename t_node::attr_type;
  typename t_node::edge_type;
  
  requires std::derived_from<
    t_node, detail::basic_graph_node<
      typename t_node::key_type, 
      typename t_node::attr_type, 
      typename t_node::edge_type
    >
  >;
};
// clang-format on

template <graph_node t_node> using graph_node_key_t = typename t_node::key_type;
template <graph_node t_node> using graph_node_edge_t = typename t_node::edge_type;
template <graph_node t_node> using graph_node_attr_t = typename t_node::attr_type;

template <graph_node node_t, detail::hasher<typename node_t::value_type>> class directed_graph;

// clang-format off
template <typename t_graph>
concept graph = requires() {
  requires std::derived_from<t_graph, directed_graph<typename t_graph::node_type, typename t_graph::hash_type>>;
};
// clang-format on

template <graph> class breadth_first_traversal;

template <graph_node t_node, detail::hasher<typename t_node::value_type> t_hash> class directed_graph {
public:
  using node_type = t_node;

  using key_type = typename node_type::key_type;
  using value_type = typename node_type::value_type;
  using size_type = std::size_t;

  using hash_type = t_hash;

private:
  std::unordered_map<value_type, node_type, hash_type> m_adj_list;
  size_type m_edge_n = 0;

public:
  directed_graph() = default;

  virtual ~directed_graph() {}

  // inserts vertex
  virtual bool insert(const value_type &val) { return insert_base(val); }

  // inserts edge from first to second
  virtual bool insert(const key_type &first, const key_type &second) {
    if (!contains(first)) insert(first);
    if (!contains(second)) insert(second);
    return insert_base(first, second);
  }

  // checks if edge from first to second exists
  bool contains(const key_type &first, const key_type &second) const {
    if (!(m_adj_list.contains(first) && m_adj_list.contains(second))) return false;
    auto &&list = m_adj_list.at(first);
    if (std::find(list.begin(), list.end(), second) == list.end()) return false;
    return true;
  }

  // checks if vertex exists
  bool contains(const key_type &val) const { return m_adj_list.contains(val); }

  // returns number of edges in the graph
  size_type edges() const { return m_edge_n; }

  // returns number of vertices in the graph
  size_type vertices() const { return m_adj_list.size(); }

  // returns true if graph is empty
  bool empty() const { return !vertices(); }

  // Returns number of val's successors in the graph
  size_type successors(const key_type &val) const {
    if (!m_adj_list.contains(val)) throw std::logic_error{"Attempt to get number of successors of non-existent vertex"};
    return m_adj_list[val].size();
  }

  // returns true if first is directly connected to second
  bool connected(const key_type &first, const key_type &second) const {
    if (!(m_adj_list.contains(first) && m_adj_list.contains(second))) return false;
    auto &&first_node = m_adj_list.at(first);
    auto &&found = std::find(first_node.begin(), first_node.end(), second);
    if (found == first_node.end()) return false;
    return true;
  }

  // returns true if second is reachable from the first
  bool reachable(const key_type &first, const key_type &second) const {
    breadth_first_traversal search{*this};
    return search(first, [&second](auto &&val) { return val == second; });
  }

  auto find(const key_type &val) const { return m_adj_list.find(val); }

  auto begin() { return m_adj_list.begin(); }
  auto end() { return m_adj_list.end(); }
  auto begin() const { return m_adj_list.cbegin(); }
  auto end() const { return m_adj_list.cend(); }
  auto cbegin() const { return m_adj_list.cbegin(); }
  auto cend() const { return m_adj_list.cend(); }

protected:
  // insertes vertex if they are not already in the graph
  bool insert_base(const value_type &val) {
    auto &&[iter, inserted] = m_adj_list.insert({val, t_node{val}});
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

template <typename T, detail::hasher<T> hash_t = std::hash<T>>
using basic_directed_graph = directed_graph<detail::basic_graph_node<T, void, void>, hash_t>;

template <graph graph_t> class breadth_first_traversal final {
  const graph_t &m_graph;

  using key_type = typename graph_t::key_type;
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
  template <typename F> auto operator()(const value_type &root_val, F func) const {
    if (!m_graph.contains(root_val)) throw std::logic_error{"Non-existing vertex root in BFS"};

    std::unordered_map<value_type, bfs_node, typename graph_t::hash_type> nodes;
    auto &&root_node = nodes.insert({root_val, {}}).first->second;
    root_node.m_color = color_t::E_GRAY;
    root_node.m_dist = 0;

    std::deque<value_type> que;
    que.push_back(root_val);
    while (!que.empty()) {
      auto &&curr = que.front();

      if constexpr (std::convertible_to<std::invoke_result_t<F, key_type>, bool>) {
        if (func(curr)) return true;
      } else {
        func(curr);
      }

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

    if constexpr (std::convertible_to<std::invoke_result_t<F, key_type>, bool>) {
      return false;
    }
  }
};

template <graph graph_t> std::vector<typename graph_t::value_type> recursive_topo_sort(graph_t &graph) {
  using value_type = typename graph_t::value_type;
  enum class color_t {
    E_WHITE,
    E_GRAY,
    E_BLACK
  };

  struct bfs_node {
    unsigned m_finish = std::numeric_limits<unsigned>::max();
    color_t m_color = color_t::E_WHITE;
    bfs_node *m_prev = nullptr;
  };

  int time = 0;
  std::vector<value_type> scheduled;
  std::unordered_map<value_type, bfs_node, typename graph_t::hash_type> nodes;

  for (auto &&val : graph)
    nodes.insert({val.first, {}});

  const auto dfs_visit = [&time, &nodes, &graph, &scheduled](const value_type &val, auto &&dfs_visit) -> void {
    ++time;
    auto &&cur_node = nodes.at(val);
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

  return scheduled;
}

} // namespace graphs