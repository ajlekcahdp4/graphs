#include "graphs/taocp_graph.hpp"

#include <iostream>

int main() {
  graphs::punch_card_graph g{
      {1, 2},
      {1, 3},
      {2, 3},
      {2, 4},
      {3, 4}
  };
  g.dump(std::cout);
}