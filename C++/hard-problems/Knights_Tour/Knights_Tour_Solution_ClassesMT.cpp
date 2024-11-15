#include <vector>    // std::vector, std::size_t
#include <future>    // std::async
#include <algorithm> // std::sort
#include <cstdlib>   // EXIT_SUCCESS

#include "Knights_Tour_Solution_Classes_Common.hpp"

int main()
{
    constexpr size_t BOARD_SIZE = 6;
    std::vector<std::future<std::vector<KnightsTourSolver<BOARD_SIZE>::solution_t>>> hdls;
    for (size_t i = 0; i < BOARD_SIZE; i++)
    {
        for (size_t j = 0; j < BOARD_SIZE; j++)
        {
            auto fn = [i, j]()
            {
                KnightsTourSolver<BOARD_SIZE> kts(i, j);
                return kts.solve();
            };
            hdls.emplace_back(std::async(std::launch::async, fn));
        }
    }

    std::vector<KnightsTourSolver<BOARD_SIZE>::solution_t> solutions;
    for (auto &hdl : hdls)
    {
        auto res = hdl.get();
        extend(solutions, res);
    }
    std::sort(solutions.begin(), solutions.end());
    for (const auto &sol : solutions)
    {
        std::cout << sol << '\n';
    }

    return EXIT_SUCCESS;
}