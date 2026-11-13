"""
Stores the elevation model of the terrain for choosing suitable dig area.
"""
import numpy as np


class OrbitalDem:
    def __init__(self, info: dict) -> None:
        self.rows: int = int(info["rows"])            # rows in DEM
        self.cols: int = int(info["cols"])            # cols in DEM
        self.res: float = float(info["resolution"])   # DEM resolution
        self.x0: float = float(info["origin_x"])      # x origin
        self.y0: float = float(info["origin_y"])      # y origin
        z = np.array([np.nan if elev is None else elev for elev in info["data"]], dtype=float)  
        self.z: np.ndarray = z.reshape(self.rows, self.cols)   # matrix containing all elevations
        gy, gx = np.gradient(self.z, self.res)
        self.slope: np.ndarray = np.degrees(np.arctan(np.hypot(gx, gy)))

    def cell(self, x, y):
        cell_col = int(round((x - self.x0) / self.res))
        cell_row = int(round((y - self.y0) / self.res))
        if 0 <= cell_row < self.rows and 0 <= cell_col < self.cols:
            return cell_row, cell_col
        return None

    def steep_mask(self, X, Y, max_deg):
        """Boolean array (same shape as X, Y): True where the slope exceeds max_deg."""
        COLS = np.rint((X - self.x0) / self.res).astype(int) # matrix of columns center 
        ROWS = np.rint((Y - self.y0) / self.res).astype(int) # matrix of rows center
        inside = (ROWS >= 0) & (ROWS < self.rows) & (COLS >= 0) & (COLS < self.cols) #bool mask
        slope_map = np.full(X.shape, np.nan)
        slope_map[inside] = self.slope[ROWS[inside], COLS[inside]]
        return np.nan_to_num(slope_map, nan=0.0) > max_deg
