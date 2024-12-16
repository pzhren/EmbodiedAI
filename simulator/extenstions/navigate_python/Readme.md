### HeightMap Class

- **info**: Information about the height map, including width, height, resolution, X/Y coordinates, etc., as a string. Example: `'w_128h_128r_0.060000X_-4.020000Y_-3.480000'`.
- **height**: Raw data of the height map, represented as a floating-point array of size `[height*width,]`, where `height` is the number of rows and `width` is the number of columns.

## Methods

### `make_map()`
- **Purpose**: Generates a new height map based on the raw height map. The returned height map contains:
  - `0`: Represents free space.
  - `1`: Represents obstacles.
  - `2`: Represents unknown areas.
- **Returns**: The height map as a `np(int)` array.

### `de_noising()`
- **Purpose**: Removes isolated noise from the height map.

### `map_plot()`
- **Purpose**: Visualizes the height map.

### `show_info()`
- **Purpose**: Displays relevant information about the height map (e.g., width, height, resolution, X/Y coordinates).

### `save_pkl(k)`
- **k**: A keyword for the file name.
- **Purpose**: Saves the height map as a `.pkl` file.

### `compute_range()`
- **Purpose**: Calculates the range of the height map in world coordinates.
- **Returns**: `[min_x, max_x, min_y, max_y]` as a `list(float)`.

### `get_map()`
- **Purpose**: Returns the height map data.
- **Returns**: The height map as a `np(int)` array.

---

### Navigator Class

- **area_range**: The range of the global height map in world coordinates, represented as a list. Example: `[min_x, max_x, min_y, max_y]`.
- **map**: Global height map data, represented as an `array`.

## Inner Class: Planner

### `reset()`
- **Purpose**: Resets the map, cost map, and related data.

### `compute_cost_map()`
- **Purpose**: Computes a global cost map based on the global height map.

### `update_map(u_map, u_range)`
- **u_map**: Local height map, represented as an `array(int)`.
- **u_range**: Range of the local height map, represented as a `list(float)`. Example: `[min_x, max_x, min_y, max_y]`.
- **Purpose**: Updates the global height map and the global cost map using the local height map.

### `map2real(pos)`
- **pos**: Map coordinates, represented as `array(int)`.
- **Purpose**: Converts map coordinates to world coordinates.
- **Returns**: World coordinates as a `tuple(float)`.

### `real2map(pos)`
- **pos**: World coordinates, represented as `array(float)`.
- **Purpose**: Converts world coordinates to map coordinates.
- **Returns**: Map coordinates as a `tuple(int)`.

## Methods

### `navigate(goal, pos)`
- **goal**: Target coordinates in world coordinates, represented as `(float, float)`.
- **pos**: Current agent position in world coordinates, represented as `(float, float)`.
- **Purpose**: Computes a navigation path based on the goal and the current position.
- **Returns**: 
  - `path`: Navigation path in world coordinates as a `list(float)`.
  - `map_path`: Navigation path in map coordinates as a `list(int)`.

### `navigate_(goal, pos)`
- **goal**: Target coordinates in map coordinates, represented as `(int, int)`.
- **pos**: Current agent position in world coordinates, represented as `(float, float)`.
- **Purpose**: Computes a navigation path based on the goal and the current position.
- **Returns**: 
  - `path`: Navigation path in world coordinates as a `list(float)`.
  - `map_path`: Navigation path in map coordinates as a `list(int)`.

### `_navigate_(goal, pos)`
- **goal**: Target coordinates in map coordinates, represented as `(int, int)`.
- **pos**: Current agent position in map coordinates, represented as `(int, int)`.
- **Purpose**: Computes a navigation path based on the goal and the current position.
- **Returns**: 
  - `path`: Navigation path in world coordinates as a `list(float)`.
  - `map_path`: Navigation path in map coordinates as a `list(int)`.

---

### Functions

## `show_map_(map, path=None, goal)`
- **map**: The map to be visualized, represented as an `array` of size `[width*height,]`.
- **path** (optional): Navigation path in map coordinates as a `list(int)`.
- **goal** (optional): Target point in map coordinates as a `list[int, int]`.
- **Purpose**: Visualizes the map along with the optional navigation path and target point.