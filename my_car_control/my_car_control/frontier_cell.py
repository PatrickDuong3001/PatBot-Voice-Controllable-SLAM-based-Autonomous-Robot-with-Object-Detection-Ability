#Make the Frontier_Cell class set compatible
class Frontier_Cell():
    def __init__(self, grid_x: float, grid_y: float, o_x: float, o_y: float, grid_res: float):
        self.grid_x = grid_x #column of the frontier
        self.grid_y = grid_y #row of the frontier
        self.o_x = o_x
        self.o_y = o_y
        self.grid_res = grid_res

    def __eq__(self, other_cell):
        #check if other frontier cell is same as current frontier cell
        if not isinstance(other_cell, Frontier_Cell):
            return NotImplemented
        return self.grid_x == other_cell.grid_x and self.grid_y == other_cell.grid_y 

    def __hash__(self):
        #hash a cell based on its grid_x and grid_y
        return hash((self.grid_x, self.grid_y)) #pass a tuple

    def get_real_X(self) -> float:
        return self.o_x + (self.grid_x + 0.5) * self.grid_res

    def get_real_Y(self) -> float:
        return self.o_y + (self.grid_y + 0.5) * self.grid_res

    def get_grid_x(self) -> int:
        return self.grid_x

    def get_grid_y(self) -> int:
        return self.grid_y