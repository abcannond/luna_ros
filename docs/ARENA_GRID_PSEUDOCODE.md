# Arena grid / zone logic (pseudocode)

This document is meant to be a **simple mental model** of how `luna_nav` decides:

- a **coarse named zone** (starting/excavation/obstacle/construction/outside)
- a **grid-cell label** (`Gk`, `Em`, `border:...`, `outside`, `outside_arena`)

It mirrors the real code in:

- `ros2_ws/src/luna_nav/luna_nav/arena_grid.py`
- `ros2_ws/src/luna_nav/luna_nav/zone_publisher.py`
- `ros2_ws/src/luna_nav/luna_nav/arena_grid_publisher.py`

---

## Big picture: there are 2 independent “outputs”

### Output A: coarse zone name (simple rectangles)
Used by: `zone_publisher.py` → publishes `/current_zone`

This checks the robot’s **actual (x,y)** against a few rectangles in `arena_zones.yaml`.

### Output B: grid cell label (square grid + zone codes at cell center)
Used by: `arena_grid_publisher.py` → publishes `/arena_grid_cell` (and markers)

This first maps the robot’s (x,y) to a **grid cell**, then decides a label for that **cell**.

---

## Call graph (who calls who)

### Coarse zone publisher path

```text
ZonePublisherNode._on_odom(msg)
  x,y = msg.pose.pose.position.(x,y)
  zone = get_coarse_zone_name(x, y, yaml_data)
  print when zone changes
  publish zone to /current_zone
```

### Grid publisher path

```text
ArenaGridPublisherNode.__init__()
  yaml_data = load_arena_config(arena_zones.yaml)
  grid = ArenaGrid.from_config(yaml_data)     <-- precomputes grid shape + E-indexing

ArenaGridPublisherNode._on_odom(msg)
  x,y = msg.pose.pose.position.(x,y)
  label = label_for_pose(x, y, yaml_data, grid)
  print when label changes (also prints coarse zone for context)
  publish label to /arena_grid_cell
```

---

## Pseudocode: config loading

```text
load_arena_config(path):
  return YAML.parse_file(path)
```

---

## Pseudocode: how the arena is split into grid cells

Inputs (from `arena_zones.yaml`):

```text
arena bounds: x_min, x_max, y_min, y_max
cell size:    s = grid.cell_size_m
```

```text
from_config(yaml_data):
  arena = (x_min, x_max, y_min, y_max)

  x_span = x_max - x_min
  y_span = y_max - y_min

  ncols = ceil(x_span / s)
  nrows = ceil(y_span / s)

  # Optional: precompute which cells overlap excavation to assign E1..Em
  excavation_rect = zones.excavation_zone.rectangle
  e_index_by_cell = empty_map
  m = 0

  for each cell (ci in [0..ncols-1], ri in [0..nrows-1]):
    cell_rect = [x_min+ci*s, x_min+(ci+1)*s] × [y_min+ri*s, y_min+(ri+1)*s]
    cell_rect = clip_to_arena_bounds(cell_rect, arena)
    if intersection_area(cell_rect, excavation_rect) > 0:
      m += 1
      e_index_by_cell[(ci,ri)] = m

  return grid(arena, s, ncols, nrows, e_index_by_cell, prefer_excavation_e_label)
```

---

## Pseudocode: coarse zone name (rectangles only)

This is the “simple” one.

```text
get_coarse_zone_name(x, y, yaml_data):
  # origin special case
  if |x| <= origin_tolerance AND |y| <= origin_tolerance:
    return "starting zone"

  # check zones in priority order from yaml_data.zone_priority
  for zone_name in zone_priority:
    if zone_name == "obstacle_zone":
      inside_main    = point_in_rect(x, y, obstacle.rectangle_main)
      inside_exclude = point_in_rect(x, y, obstacle.rectangle_exclude)
      if inside_main AND NOT inside_exclude:
        return obstacle.coarse_name
    else:
      if point_in_rect(x, y, zones[zone_name].rectangle):
        return zones[zone_name].coarse_name

  return "outside bounds"
```

---

## Pseudocode: grid-cell label (what `arena_grid_publisher` publishes)

Key idea: the label is based on the **cell center**, so the label stays stable anywhere inside that cell.

```text
label_for_pose(x, y, yaml_data, grid):
  if (x,y) is outside grid.arena bounds:
    return outside_arena_label

  (ci, ri) = pose_cell_indices(x, y, grid.arena, grid.cell_size_m)
  if no cell:
    return outside_arena_label

  (cx, cy) = center_of_cell(ci, ri)

  codes = zone_codes_at_point(cx, cy)   # set like {"e"} or {"s","e"} or {}

  if size(codes) >= 2:
    return "border:" + join_sorted(codes, "&")   # e.g. border:e&s
  if size(codes) == 0:
    return outside_named_label                  # inside arena but not inside any named zone

  # exactly one code -> return a cell index label
  gk = ri * grid.ncols + ci + 1                 # row-major indexing

  if grid.prefer_excavation_e_label AND (ci,ri) in grid.e_index_by_cell:
    return "E" + grid.e_index_by_cell[(ci,ri)]  # E1, E2, ...
  else:
    return "G" + gk                              # G1, G2, ...
```

---

## Pseudocode: how zone codes are determined

This returns all zone codes that contain a point. (That’s what makes `border:` possible.)

```text
zone_codes_at_point(x, y, yaml_data):
  codes = empty_set

  if near_origin(x,y):
    add "s"

  if point_in_rect(x,y, starting_zone.rectangle):    add "s"
  if point_in_rect(x,y, excavation_zone.rectangle):  add "e"

  if point_in_rect(x,y, obstacle.main) AND NOT point_in_rect(x,y, obstacle.exclude):
    add "o"

  if point_in_rect(x,y, construction_zone.rectangle): add "c"

  return codes
```

---

## If you want this to feel “simpler”

The easiest simplification (behavior change) would be:

- decide codes based on the **robot’s (x,y)** instead of the **cell center (cx,cy)**, or
- remove `border:` and return just the first matching zone (like coarse zones), or
- remove `E{m}` and always return `G{k}`.

Those are design choices; the current code is doing extra work mainly to keep labels **stable per cell** and to support **border detection**.

