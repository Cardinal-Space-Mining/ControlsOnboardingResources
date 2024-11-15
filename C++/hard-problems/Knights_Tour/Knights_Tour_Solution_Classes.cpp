#include <algorithm> // std::sort
#include <vector>    // std::vector
#include <cstdlib>   // EXIT_SUCCESS

#include "Knights_Tour_Solution_Classes_Common.hpp"

int main()
{
    constexpr std::size_t WIDTH = 5;
    std::vector<KnightsTourSolver<WIDTH>::solution_t> solutions;
    for (size_t i = 0; i < WIDTH; i++)
    {
        for (size_t j = 0; j < WIDTH; j++)
        {
            KnightsTourSolver<WIDTH> kts(i, j);
            auto these_sols = kts.solve();
            extend(solutions, these_sols);
        }
    }
    std::sort(solutions.begin(), solutions.end());
    for (const auto &sol : solutions)
    {
        std::cout << sol << '\n';
    }

    return EXIT_SUCCESS;
}