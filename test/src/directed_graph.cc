/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <tsimmerman.ss@phystech.edu>, <alex.rom23@mail.ru> wrote this file.  As long
 * as you retain this notice you can do whatever you want with this stuff. If we
 * meet some day, and you think this stuff is worth it, you can buy me a beer in
 * return.
 * ----------------------------------------------------------------------------
 */

#include "graphs/directed_graph.hpp"

#include <gtest/gtest.h>

using directed_graph = graphs::basic_directed_graph<int>;

TEST(test_directed_graph, test_insert_vertex) {
  directed_graph A;
  EXPECT_EQ(A.vertices(), 0);
  EXPECT_TRUE(A.insert(1));
  EXPECT_TRUE(A.insert(2));
  EXPECT_FALSE(A.insert(1));
  EXPECT_EQ(A.vertices(), 2);
  EXPECT_TRUE(A.contains(1));
  EXPECT_TRUE(A.contains(2));
  EXPECT_FALSE(A.contains(4));
}

TEST(test_directed_graph, test_insert_edge) {
  directed_graph A;
  EXPECT_TRUE(A.insert(1, 2));
  EXPECT_TRUE(A.insert(1, 3));
  EXPECT_TRUE(A.insert(3, 2));
  EXPECT_TRUE(A.insert(3, 3));
  EXPECT_FALSE(A.insert(1, 2));
  EXPECT_EQ(A.edges(), 4);
  EXPECT_EQ(A.vertices(), 3);
  EXPECT_TRUE(A.contains(1));
  EXPECT_TRUE(A.contains(2));
  EXPECT_TRUE(A.contains(3));
  EXPECT_TRUE(A.contains(1, 2));
  EXPECT_TRUE(A.contains(1, 3));
  EXPECT_TRUE(A.contains(3, 2));
  EXPECT_TRUE(A.contains(3, 3));
}

TEST(test_directed_graph, test_connected) {
  directed_graph A;
  A.insert(3, 6);
  A.insert(3, 5);
  A.insert(5, 4);
  A.insert(4, 2);
  A.insert(2, 5);
  A.insert(1, 2);
  A.insert(1, 4);
  A.insert(1, 1);

  EXPECT_TRUE(A.connected(3, 5));
  EXPECT_TRUE(A.connected(1, 2));
  EXPECT_TRUE(A.connected(1, 1));
  EXPECT_FALSE(A.connected(1, 6));
  EXPECT_FALSE(A.connected(1, 3));
  EXPECT_FALSE(A.connected(5, 3));
}

TEST(test_directed_graph, test_reachable) {
  directed_graph A;
  A.insert(3, 6);
  A.insert(3, 5);
  A.insert(5, 4);
  A.insert(4, 2);
  A.insert(2, 5);
  A.insert(1, 2);
  A.insert(1, 4);
  A.insert(1, 1);

  EXPECT_TRUE(A.reachable(3, 6));
  EXPECT_TRUE(A.reachable(3, 2));
  EXPECT_FALSE(A.reachable(3, 1));
}

TEST(test_directed_graph, test_BFS) {
  directed_graph A;
  A.insert(3, 6);
  A.insert(3, 5);
  A.insert(5, 4);
  A.insert(4, 2);
  A.insert(2, 5);
  A.insert(1, 2);
  A.insert(1, 4);
  graphs::breadth_first search{A};
  EXPECT_TRUE(search(3, [](auto &&val) { return val == 2; }));
  EXPECT_TRUE(search(2, [](auto &&val) { return val == 5; }));
  EXPECT_FALSE(search(2, [](auto &&val) { return val == 3; }));
  EXPECT_FALSE(search(4, [](auto &&val) { return val == 11; }));
}