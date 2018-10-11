from samba.dcerpc.nbt import name

from BaseAI import BaseAI
from math import log

class PlayerAI(BaseAI):
    def getMove(self, grid):
        value, best_move = self.minimax(grid, 5, float("-inf"), float("+inf"), True)

        return best_move

    def score(self, grid):
        number_of_available_cells = len(grid.getAvailableCells())

        if number_of_available_cells > 0:
            available_cell_score = log(number_of_available_cells)
        else:
            available_cell_score = 0

        return 0.1 * self.smoothness(grid) + 1.0 * self.monotonicity(grid) + 2.7 * available_cell_score + 1.0 * log(grid.getMaxTile()) / log(2)

    """ Score a particular grid. """
    def score_old(self, grid):
        total_score = 0

        for x in range(4):
            for y in range(4):
                value = grid.getCellValue((x, y))

                if value != None:
                    total_score += value

        cells_occupied = 16 - len(grid.getAvailableCells())

        return total_score / cells_occupied

    def smoothness(self, grid):
        smoothness = 0.0

        for x in range(4):
            for y in range(4):
                cell_value = grid.getCellValue((x, y))

                if cell_value != 0:
                    value = log(cell_value) / log(2)

                    # Right and down
                    for direction in [(1, 0), (0, 1)]:
                        target_cell = self.find_free_position_farthest_away(grid, (x, y), direction)[1]

                        if not grid.crossBound(target_cell):
                            target_cell_value = grid.getCellValue(target_cell)

                            if target_cell_value != 0:
                                smoothness -= abs(value - log(target_cell_value) / log(2))

        return smoothness

    def monotonicity(self, grid):
        totals = [0, 0, 0, 0];

        for x in range(4):
            current = 0
            next = current + 1

            while next < 4:
                while next < 4 and grid.getCellValue((x, next)) != 0:
                    next += 1

                if next >= 4:
                    next -= 1

                current_cell_value = grid.getCellValue((x, current))
                current_value = 0

                if current_cell_value != 0:
                    current_value = log(current_cell_value) / log(2)

                next_cell_value = grid.getCellValue((x, next))
                next_value = 0

                if next_cell_value != 0:
                    next_value = log(next_cell_value) / log(2)

                if current_value > next_value:
                    totals[0] += next_value - current_value
                elif next_value > current_value:
                    totals[1] += current_value - next_value

                current = next
                next += 1

        for y in range(4):
            current = 0
            next = current + 1

            while next < 4:
                while next < 4 and grid.getCellValue((next, y)) != 0:
                    next += 1

                if next >= 4:
                    next -= 1

                current_cell_value = grid.getCellValue((current, y))
                current_value = 0

                if current_cell_value != 0:
                    current_value = log(current_cell_value) / log(2)

                next_cell_value = grid.getCellValue((next, y))
                next_value = 0

                if next_cell_value != 0:
                    next_value = log(next_cell_value) / log(2)

                if current_value > next_value:
                    totals[2] += next_value - current_value
                elif next_value > current_value:
                    totals[3] += current_value - next_value

                current = next
                next += 1

        return max(totals[0], totals[1]) + max(totals[2], totals[3])


    def find_free_position_farthest_away(self, grid, position, direction):
        found = False
        previous = None

        while not found:
            previous = position
            position = (previous[0] + direction[0], previous[1] + direction[1])

            found = grid.crossBound(position) or grid.getCellValue(position) != None

        return (previous, position)

    def minimax(self, grid, depth, alpha, beta, maximizing_player):
        if depth == 0 or not grid.canMove():
            return (self.score(grid), None)

        if maximizing_player:
            value = float("-inf")
            best_move = None

            for move in grid.getAvailableMoves():
                new_grid = grid.clone()
                new_grid.move(move)

                value = max(value, self.minimax(new_grid, depth - 1, alpha, beta, False)[0])

                if value > alpha:
                    alpha = value
                    best_move = move

                if alpha >= beta:
                    break

            return (value, best_move)
        else:
            value = float("+inf")

            for cell in grid.getAvailableCells():
                new_grid = grid.clone()
                new_grid.insertTile(cell, 2)

                value = min(value, self.minimax(new_grid, depth - 1, alpha, beta, True)[0])

                break

                beta = min(beta, value)

                if alpha >= beta:
                    break

            return (value, None)

