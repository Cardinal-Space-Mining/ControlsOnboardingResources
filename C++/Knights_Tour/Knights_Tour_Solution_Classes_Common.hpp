#pragma once
#include <array>    // std::array
#include <vector>   // std::vector, std::size_t
#include <iomanip>  // std::setw
#include <iostream> // std::ostream
#include <cstring>  // std::memcmp
#include <cstdint>  // std::int32_t

// This just takes all the items in from and moves them into dst
template <class VecCtnrType>
void extend(std::vector<VecCtnrType> &dst, std::vector<VecCtnrType> &from)
{
    dst.reserve(dst.size() + from.size());
    for (auto &item : from)
    {
        dst.emplace_back(std::move(item));
    }
    from.clear();
}

template <std::size_t BOARD_SIZE>
class KnightsTourSolver
{
public:
    // We are defining a consistant integer type to use inside our class. Conversions can get expensive
    using itype = std::int_fast32_t;

    // BOARD_SIZE by BOARD_SIZE chessboard. We just need to track where we have been
    using board_t = std::array<std::array<bool, BOARD_SIZE>, BOARD_SIZE>;

    // This is just an array with functions to sort and print the array
    typedef class Solution : public std::array<itype, BOARD_SIZE * BOARD_SIZE>
    {
        // Prints a solution
        friend std::ostream &operator<<(std::ostream &os, const Solution &solution)
        {
            os << '[';
            for (size_t i = 0; i < solution.size() - 1; i++)
            {
                os << std::setw(2) << solution[i] << ", ";
            }
            os << std::setw(2) << solution[solution.size() - 1] << ']';
            return os;
        }

        friend bool operator==(const Solution &a, const Solution &b)
        {
            return std::memcmp(a.data(), b.data(), sizeof(a)) == 0;
        }

        friend bool operator<(const Solution &a, const Solution &b)
        {
            return std::memcmp(a.data(), b.data(), sizeof(a)) < 0;
        }

        friend bool operator!=(const Solution &c1, const Solution &c2) { return !(operator==(c1, c2)); }
        friend bool operator>(const Solution &c1, const Solution &c2) { return operator<(c2, c1); }

        friend bool operator<=(const Solution &c1, const Solution &c2) { return !(operator>(c1, c2)); }
        friend bool operator>=(const Solution &c1, const Solution &c2) { return !(operator<(c1, c2)); }
    } solution_t;

    // This is just a vector of solutions
    using solutions_t = std::vector<solution_t>;

private:
    // a list of all the possible moves a knight can make
    static constexpr std::array<std::array<itype, 2>, 8> knight_moves{{{-2, -1}, {-2, 1}, {2, -1}, {2, 1}, {-1, -2}, {-1, 2}, {1, -2}, {1, 2}}};

public:
    // Solver constructor. Dictates which square of the chessboard the solver will take
    KnightsTourSolver(itype x, itype y) : start_x(x), start_y(y) {}

    // finds all the knights tours that start at the start_x and start_y location
    solutions_t solve() noexcept
    {
        this->tour(start_x, start_y);
        return this->solutions;
    };

private:
    // Converts an x-y coordinate into a single integer value according to the specification in the readme
    static constexpr itype index(itype x, itype y) noexcept
    {
        return (BOARD_SIZE * x) + y + 1;
    }

    // Returns true if the knight can make a given move
    inline bool is_valid_move(itype x, itype y, const std::array<itype, 2> &move) const noexcept
    {
        itype dst_x = x + move[0];
        itype dst_y = y + move[1];
        return dst_x >= 0 && dst_x < BOARD_SIZE && dst_y >= 0 && dst_y < BOARD_SIZE && !board[dst_x][dst_y];
    }

    // Recursive implementation
    void tour(itype x, itype y) noexcept
    {
        order[visited++] = index(x, y);
        board[x][y] = true;

        if (visited == (BOARD_SIZE * BOARD_SIZE))
        {
            this->solutions.push_back(order);
        }

// this pragma tells the compiler to unroll the loop
#pragma unroll
        for (const auto &move : knight_moves)
        {
            if (is_valid_move(x, y, move))
            {
                tour(x + move[0], y + move[1]);
            }
        }

        board[x][y] = false;
        visited--;
    }

private:
    itype start_x, start_y;
    board_t board{};
    itype visited = 0;
    solution_t order{};
    std::vector<solution_t> solutions;
};