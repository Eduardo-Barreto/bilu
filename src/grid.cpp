#include "bilu/grid.hpp"

#include <fstream>
#include <iostream>

namespace bilu {

Grid::Grid(int rows, int cols) : rows(rows), cols(cols) {
    init(rows, cols);
}

void Grid::init(int rows, int cols) {
    exploredMap.resize(rows, std::vector<CellState>(cols, UNKNOWN));
}

bool Grid::isValid(int x, int y) const {
    return x >= 0 && x < rows && y >= 0 && y < cols;
}

void Grid::updateExploredCell(int x, int y, CellState cell_state) {
    if (isValid(x, y)) {
        exploredMap[x][y] = cell_state;
    }
}

CellState Grid::getExploredCellState(int x, int y) const {
    if (isValid(x, y)) {
        return exploredMap[x][y];
    }
    return OBSTACLE;
}

void Grid::displayExploredMap(int currentX, int currentY) const {
    auto display = exploredMap;
    display[currentX][currentY] = START;

    for (const auto& row : display) {
        for (const auto& cell : row) {
            switch (cell) {
                case UNKNOWN:
                    std::cout << '?';
                    break;
                case FREE:
                    std::cout << '.';
                    break;
                case OBSTACLE:
                    std::cout << '#';
                    break;
                case START:
                    std::cout << 'R';
                    break;
                case END:
                    std::cout << 'E';
                    break;
            }
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

void Grid::saveExploredMap(const std::string& filename) {
    std::ofstream outFile(filename);
    if (outFile) {
        for (const auto& row : exploredMap) {
            for (size_t i = 0; i < row.size(); ++i) {
                switch (row[i]) {
                    case UNKNOWN:
                        outFile << "?";
                        break;
                    case FREE:
                        outFile << ".";
                        break;
                    case OBSTACLE:
                        outFile << "#";
                        break;
                    case START:
                        outFile << "R";
                        break;
                    case END:
                        outFile << "E";
                        break;
                }
                if (i != row.size() - 1) {
                    outFile << ", ";
                }
            }
            outFile << '\n';
        }
    }
}

void Grid::loadExploredMap(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile) {
        return;
    }

    std::string line;
    int         row = 0;
    while (std::getline(inFile, line) && row < rows) {
        size_t col = 0;
        size_t startPos = 0;
        size_t endPos = line.find(", ");
        while (endPos != std::string::npos && static_cast<int>(col) < cols) {
            std::string cell = line.substr(startPos, endPos - startPos);
            if (cell == "?") {
                exploredMap[row][col] = UNKNOWN;
            } else if (cell == ".") {
                exploredMap[row][col] = FREE;
            } else if (cell == "#") {
                exploredMap[row][col] = OBSTACLE;
            } else if (cell == "R") {
                exploredMap[row][col] = START;
            } else if (cell == "E") {
                exploredMap[row][col] = END;
            }

            startPos = endPos + 2;
            endPos = line.find(", ", startPos);
            ++col;
        }
        ++row;
    }
}

}  // namespace bilu
