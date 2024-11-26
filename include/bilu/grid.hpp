#ifndef GRID_HPP
#define GRID_HPP

#include <string>
#include <vector>

namespace bilu {

enum CellState {
    UNKNOWN,
    FREE,
    OBSTACLE,
    START,
    END
};

class Grid {
public:
    Grid() = default;

    Grid(int rows, int cols);

    void init(int rows, int cols);

    bool isValid(int x, int y) const;

    void updateExploredCell(int x, int y, CellState cell_state);

    CellState getExploredCellState(int x, int y) const;

    void displayExploredMap(int currentX, int currentY) const;

    void saveExploredMap(const std::string& filename);

    void loadExploredMap(const std::string& filename);

private:
    int                                 rows{}, cols{};
    std::vector<std::vector<CellState>> exploredMap;
};

}  // namespace bilu
#endif  // GRID_HPP
