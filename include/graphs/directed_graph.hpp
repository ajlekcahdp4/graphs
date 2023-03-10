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
  using entry_type = std::pair<key_type, edge_type>;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key_value(value_type &val) { return val.first; }
  static const key_type &get_key_value(const value_type &val) { return val.first; }

  static key_type &get_key_entry(entry_type &entry) { return entry.first; }
  static const key_type &get_key_entry(const entry_type &entry) { return entry.first; }
};

template <typename t_key, typename t_attr> struct adjacency_list_traits<t_key, t_attr, void> {
  using key_type = t_key;
  using attr_type = t_attr;

  using value_type = std::pair<key_type, attr_type>;
  using entry_type = key_type;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key_value(value_type &val) { return val.first; }
  static const key_type &get_key_value(const value_type &val) { return val.first; }

  static key_type &get_key_entry(entry_type &entry) { return entry; }
  static const key_type &get_key_entry(const entry_type &entry) { return entry; }
};

template <typename t_key, typename t_edge> struct adjacency_list_traits<t_key, void, t_edge> {
  using key_type = t_key;
  using edge_type = t_edge;

  using value_type = key_type;
  using entry_type = std::pair<value_type, edge_type>;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key_value(value_type &val) { return val; }
  static const key_type &get_key_value(const value_type &val) { return val; }

  static key_type &get_key_entry(entry_type &entry) { return entry.first; }
  static const key_type &get_key_entry(const entry_type &entry) { return entry.first; }
};

template <typename t_key> struct adjacency_list_traits<t_key, void, void> {
  using key_type = t_key;

  using value_type = key_type;
  using entry_type = key_type;
  using type = std::vector<entry_type>;

public:
  static key_type &get_key_value(value_type &val) { return val; }
  static const key_type &get_key_value(const value_type &val) { return val; }

  static key_type &get_key_entry(entry_type &entry) { return entry; }
  static const key_type &get_key_entry(const entry_type &entry) { return entry; }
};

template <typename t_key, typename t_attr, typename t_edge>
using adjacency_list_t = typename adjacency_list_traits<t_key, t_attr, t_edge>::type;

template <typename t_key, typename t_attr, typename t_edge>
class basic_graph_node : private adjacency_list_t<t_key, t_attr, t_edge> {
public:
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

  const key_type &key() const & { return traits::get_key_value(m_val); }
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

// clang-format off
template <typename t_node>
concept graph_node = requires() {
  typename t_node::key_type;
  typename t_node::value_type;
  typename t_node::attr_type;
  typename t_node::edge_type;
  
  requires std::derived_from<
    t_node, basic_graph_node<
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
template <graph_node t_node> using graph_node_value_t = typename t_node::value_type;

// Forward declaration
template <graph_node t_node, hasher<graph_node_key_t<t_node>>, comparator<graph_node_key_t<t_node>>>
class directed_graph;

// clang-format off
template <typename t_graph, typename node_type = typename t_graph::node_type>
concept graph = requires() {
  requires graph_node<node_type>;
  requires hasher<typename t_graph::hash_type, graph_node_key_t<node_type>>;
  requires comparator<typename t_graph::comp_type, graph_node_key_t<node_type>>;
};
// clang-format on

template <graph> class breadth_first_searcher;

template <
    graph_node t_node, hasher<graph_node_key_t<t_node>> t_hash, comparator<graph_node_key_t<t_node>> t_comp,
    typename t_edge>
class directed_graph_storage {
public:
  using node_type = t_node;

protected:
  using traits = typename node_type::traits;

public:
  using key_type = typename node_type::key_type;
  using value_type = typename node_type::value_type;
  using edge_type = typename node_type::edge_type;

  using size_type = std::size_t;

  using hash_type = t_hash;
  using comp_type = t_comp;

protected:
  std::unordered_map<key_type, node_type, hash_type, comp_type> m_adj_list;
  size_type m_edge_n = 0;

protected:
  directed_graph_storage() = default;

  // Insertes a vertex if it's not contained in the graph
  bool insert(const value_type &val) {
    const auto to_insert = std::pair{traits::get_key_value(val), node_type{val}};
    const auto inserted = m_adj_list.insert(to_insert).second;
    return inserted;
  }

  // Insertes a vertex if it's not contained in the graph
  bool insert(value_type &&val) {
    const auto key = traits::get_key_value(val);
    const auto to_insert = std::pair{key, node_type{std::move(val)}};
    const auto inserted = m_adj_list.insert(to_insert).second;
    return inserted;
  }

  bool create_link(const key_type &vert1, const key_type &vert2, const edge_type &edge_attr) {
    if (auto inserted = m_adj_list.at(vert1).add_adj(std::pair{vert2, edge_attr})) {
      ++m_edge_n;
      return true;
    }
    return false;
  }

  bool create_link(const key_type &vert1, const key_type &vert2, edge_type &&edge_attr) {
    if (auto inserted = m_adj_list.at(vert1).add_adj(std::pair{vert2, std::move(edge_attr)})) {
      ++m_edge_n;
      return true;
    }
    return false;
  }

  // Check whether a vertex is present
  bool contains(const key_type &val) const { return m_adj_list.contains(val); }
};

template <graph_node t_node, hasher<graph_node_key_t<t_node>> t_hash, comparator<graph_node_key_t<t_node>> t_comp>
class directed_graph_storage<t_node, t_hash, t_comp, void> {
public:
  using node_type = t_node;

protected:
  using traits = typename node_type::traits;

public:
  using key_type = typename node_type::key_type;
  using value_type = typename node_type::value_type;
  using edge_type = typename node_type::edge_type;

  using size_type = std::size_t;

  using hash_type = t_hash;
  using comp_type = t_comp;

protected:
  std::unordered_map<key_type, node_type, hash_type, comp_type> m_adj_list;
  size_type m_edge_n = 0;

protected:
  directed_graph_storage() = default;

  // Insertes a vertex if it's not contained in the graph
  bool insert(const value_type &val) {
    const auto to_insert = std::pair{traits::get_key_value(val), node_type{val}};
    const auto inserted = m_adj_list.insert(to_insert).second;
    return inserted;
  }

  // Insertes a vertex if it's not contained in the graph
  bool insert(value_type &&val) {
    const auto key = traits::get_key_value(val);
    const auto to_insert = std::pair{key, node_type{std::move(val)}};
    const auto inserted = m_adj_list.insert(to_insert).second;
    return inserted;
  }

  bool create_link(const key_type &vert1, const key_type &vert2) {
    if (auto inserted = m_adj_list.at(vert1).add_adj(vert2)) {
      ++m_edge_n;
      return true;
    }
    return false;
  }

  // Check whether a vertex is present
  bool contains(const key_type &val) const { return m_adj_list.contains(val); }

  // Create an edge first -> second. Create corresponding vertices if necessary
  bool insert(const value_type &first, const value_type &second) {
    if (!contains(traits::get_key_value(first))) insert(first);
    if (!contains(traits::get_key_value(second))) insert(second);
    return create_link(traits::get_key_value(first), traits::get_key_value(second));
  }
};

namespace detail {

enum class bfs_color {
  E_WHITE,
  E_GRAY,
  E_BLACK
};

struct bfs_node {
  bfs_color m_color = bfs_color::E_WHITE;
};

}; // namespace detail

template <graph t_graph> using graph_key_t = typename t_graph::key_type;
template <graph t_graph> using graph_node_t = typename t_graph::node_type;
template <graph t_graph> using graph_hash_t = typename t_graph::hash_type;
template <graph t_graph> using graph_comp_t = typename t_graph::comp_type;

template <graph t_graph, std::invocable<graph_node_t<t_graph>> F>
auto breadth_first_search(const t_graph &graph, const graph_key_t<t_graph> &root, F func) {
  using graph_type = t_graph;

  using key_type = graph_key_t<graph_type>;
  using hash_type = graph_hash_t<graph_type>;
  using node_type = graph_node_t<graph_type>;
  using comp_type = graph_comp_t<graph_type>;

  using detail::bfs_color;
  using detail::bfs_node;

  if (!graph.contains(root)) throw std::out_of_range{"Search root out of range"};
  constexpr auto can_break = std::convertible_to<std::invoke_result_t<F, node_type>, bool>;

  std::unordered_map<key_type, bfs_node, hash_type, comp_type> nodes;
  nodes.insert({root, bfs_node{bfs_color::E_GRAY}});
  std::deque<const node_type *> queue;

  queue.push_back(std::addressof(graph.find(root)->second));

  while (!queue.empty()) {
    const node_type &curr = *queue.front(); // Get the reference to the queued node.
    const auto &curr_key = curr.key();

    if constexpr (can_break) {
      if (func(curr)) return true;
    } else {
      func(curr);
    }

    queue.pop_front();

    auto &curr_node = nodes.insert({curr_key, {}}).first->second;
    const auto &curr_graph_node = graph.find(curr.key())->second;

    for (const auto &adj : curr_graph_node) {
      const auto key = graph_type::key_from_entry(adj);
      auto &adj_node = nodes.insert({key, bfs_node{}}).first->second;

      if (adj_node.m_color == bfs_color::E_WHITE) {
        adj_node.m_color = bfs_color::E_GRAY;
        queue.push_back(std::addressof(graph.find(key)->second));
      }
    }

    curr_node.m_color = bfs_color::E_BLACK;
  }

  if constexpr (can_break) return false;
}

template <graph_node t_node, hasher<graph_node_key_t<t_node>> t_hash, comparator<graph_node_key_t<t_node>> t_comp>
class directed_graph : private directed_graph_storage<t_node, t_hash, t_comp, graph_node_edge_t<t_node>>,
                       private t_hash,
                       private t_comp {
  using base_type = directed_graph_storage<t_node, t_hash, t_comp, graph_node_edge_t<t_node>>;

public:
  using typename base_type::node_type;

protected:
  using typename base_type::traits;
  using entry_type = typename traits::entry_type;

public:
  using typename base_type::edge_type;
  using typename base_type::key_type;
  using typename base_type::value_type;

  using typename base_type::size_type;

  using typename base_type::comp_type;
  using typename base_type::hash_type;

protected:
  using base_type::m_adj_list;
  using base_type::m_edge_n;

public:
  directed_graph() = default;

  size_type edges() const { return m_edge_n; }             // Get number of edges
  size_type vertices() const { return m_adj_list.size(); } // Get the total number of vertices
  bool empty() const { return !vertices(); }               // Returns true if the container is empty

  auto begin() { return m_adj_list.begin(); }
  auto end() { return m_adj_list.end(); }
  auto begin() const { return m_adj_list.cbegin(); }
  auto end() const { return m_adj_list.cend(); }
  auto cbegin() const { return m_adj_list.cbegin(); }
  auto cend() const { return m_adj_list.cend(); }
  auto rbegin() { return m_adj_list.rbegin(); }
  auto rend() { return m_adj_list.rend(); }
  auto crbegin() const { return m_adj_list.crbegin(); }
  auto crend() const { return m_adj_list.crend(); }
  auto find(const key_type &val) const { return m_adj_list.find(val); }

  bool connected(const key_type &first, const key_type &second) const {
    if (!(m_adj_list.contains(first) && m_adj_list.contains(second))) return false;
    auto &&list = m_adj_list.at(first);

    return std::find_if(list.begin(), list.end(), [&](const entry_type &elem) {
             return t_comp::operator()(traits::get_key_entry(elem), second);
           }) != list.end();
  }

  using base_type::contains;
  using base_type::create_link;
  using base_type::insert;

  // Returns true if there exists a path from first -> .... -> second
  bool reachable(const key_type &first, const key_type &second) const {
    return breadth_first_search(*this, first, [&second](auto &&val) { return val.key() == second; });
  }

  // Returns number of val's successors in the graph
  size_type successors(const key_type &val) const {
    if (!m_adj_list.contains(val)) {
      throw std::out_of_range{"Key out of range"};
    }

    return m_adj_list[val].size();
  }

  static key_type &key_from_value(value_type &val) { return traits::get_key_value(val); }
  static const key_type &key_from_value(const value_type &val) { return traits::get_key_value(val); }

  static key_type &key_from_entry(entry_type &entry) { return traits::get_key_entry(entry); }
  static const key_type &key_from_entry(const entry_type &entry) { return traits::get_key_entry(entry); }
};

template <typename K, typename A, typename E, hasher<K> t_hash = std::hash<K>, comparator<K> t_comp = std::equal_to<K>>
using basic_directed_graph = directed_graph<basic_graph_node<K, A, E>, t_hash, t_comp>;

template <graph t_graph> std::vector<typename t_graph::value_type> recursive_topo_sort(t_graph &graph) {
  using value_type = typename t_graph::value_type;

  enum class node_color {
    E_WHITE,
    E_GRAY,
    E_BLACK
  };

  struct bfs_node {
    node_color m_color = node_color::E_WHITE;
  };

  std::vector<value_type> scheduled;
  std::unordered_map<value_type, bfs_node, typename t_graph::hash_type> nodes;

  for (const auto &val : graph) {
    nodes.insert({val.first, bfs_node{}});
  }

  const auto dfs_visit = [&nodes, &graph, &scheduled](const value_type &val, auto &&dfs_visit) -> void {
    auto &cur_node = nodes.at(val);
    cur_node.m_color = node_color::E_GRAY;
    auto &graph_node = graph.find(val)->second;

    for (auto &adj : graph_node) {
      auto &adj_node = nodes.at(adj);
      if (adj_node.m_color == node_color::E_WHITE) {
        dfs_visit(adj, dfs_visit);
      }
    }

    cur_node.m_color = node_color::E_BLACK;
    scheduled.push_back(val);
  };

  for (auto &&val : graph) {
    if (nodes.at(val.first).m_color != node_color::E_WHITE) continue;
    dfs_visit(val.first, dfs_visit);
  }

  return scheduled;
}

} // namespace graphs