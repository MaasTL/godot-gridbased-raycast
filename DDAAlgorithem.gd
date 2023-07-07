extends RefCounted
class_name DDAAlgorithem
# <Digital differencial analysis> Algorithem
# for determining the obstacle intersection point between two points in a grid
#
# Fast, robust and relieable
# See also: https://github.com/OneLoneCoder/olcPixelGameEngine/blob/master/Videos/OneLoneCoder_PGE_RayCastDDA.cpp

const WALK_MAX_DISTANCE: float = 100.0


static func cast_ray(point_start: Vector2, point_end: Vector2, is_cell_empty: Callable, walk_max_distance := WALK_MAX_DISTANCE):
	if point_start == point_end:
		print_debug("Can't calculate direction -> ",
		"The Start: ", point_start, " and End: ", point_end, " are the same.")
		return null
	
	var start_cell: Vector2i = Vector2i(point_start.floor()) # IMPORTANT!!! Don't remove the floor()!
	
	# Raycast
	var ray_start: Vector2 = point_start - Vector2(start_cell)
	var ray_direction: Vector2 = (point_end - point_start).normalized()
	
	var ray_unit_step_size := Vector2(
		sqrt(1 + pow(ray_direction.y / ray_direction.x, 2)),
		sqrt(1 + pow(ray_direction.x / ray_direction.y, 2))
	)
	
	var walked_cells: Vector2i = Vector2i(ray_start)
	var ray_length1D: Vector2
	var step: Vector2i = ray_direction.sign()
	
	# Establish Starting Conditions
	if ray_direction.x < 0:
		ray_length1D.x = (ray_start.x - float(walked_cells.x))
	else:
		ray_length1D.x = (float(walked_cells.x + 1) - ray_start.x)

	if ray_direction.y < 0:
		ray_length1D.y = (ray_start.y - float(walked_cells.y))
	else:
		ray_length1D.y = (float(walked_cells.y + 1) - ray_start.y)
	ray_length1D *= ray_unit_step_size
	
	# Perform "Walk" until collision or range check
	var collided_with_cell := false
	var walk_distance := 0.0
	while (not collided_with_cell and walk_distance < walk_max_distance):
		# Walk along shortest path
		if ray_length1D.x < ray_length1D.y:
			walked_cells.x += step.x;
			walk_distance = ray_length1D.x;
			ray_length1D.x += ray_unit_step_size.x;
		else:
			walked_cells.y += step.y;
			walk_distance = ray_length1D.y;
			ray_length1D.y += ray_unit_step_size.y;
		
		# Test tile at new test coordinate
		if not is_cell_empty.call(start_cell+walked_cells):
			collided_with_cell = true
	
	# Calculate intersection location
	if collided_with_cell:
		var intersection_point = point_start + ray_direction * walk_distance
		var intersection_cell = start_cell+walked_cells
		return {
			"point": intersection_point,
			"cell": intersection_cell
		}
	
	return null
