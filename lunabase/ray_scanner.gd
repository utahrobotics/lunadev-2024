class_name RayScanner
extends Camera3D


@export var width := 160
@export var height := 90
@export var distance := 3.0

var raycasts: Array[RayCast3D] = []


func _ready() -> void:
	for y in range(height):
		for x in range(width):
			var raycast := RayCast3D.new()
			var end := project_position(Vector2(x as float / width, y as float / height) * Vector2(get_viewport().size), distance)
			raycast.target_position = to_local(end)
			add_child(raycast)
			raycasts.append(raycast)


func scan() -> Array[Vector3]:
	var out: Array[Vector3] = []
	#var first := true
	for raycast in raycasts:
		if raycast.is_colliding():
			var point := raycast.get_collision_point()
			#if first:
				#first = false
				#print(point)
			#out.append(raycast.get_collision_point())
			out.append(global_transform.inverse() * point)
	return out
