class_name RayScanner
extends Camera3D


@export var width := 160
@export var height := 90
@export var distance := 3.0

var raycasts: Array[RayCast3D] = []


func _ready() -> void:
	#await get_viewport().ready
	print("[")
	for y in range(height):
		for x in range(width):
			var raycast := RayCast3D.new()
			#print(get_viewport().name)
			var end := project_position(Vector2(x as float / width, y as float / height) * Vector2(get_viewport().size), distance)
			raycast.target_position = to_local(end)
			print("\tVector3::new", raycast.target_position.normalized(), ",")
			raycast.debug_shape_thickness = 2
			add_child(raycast)
			raycasts.append(raycast)
	print("]")


func scan() -> Array[float]:
	var out: Array[float] = []
	for raycast in raycasts:
		if raycast.is_colliding():
			var point := raycast.get_collision_point()
			out.append(raycast.get_collision_point().distance_to(global_position))
		else:
			out.append(0.0)
			#out.append(global_transform.inverse() * point)
	return out
