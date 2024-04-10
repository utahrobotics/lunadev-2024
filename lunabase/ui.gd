extends Node


func _input(event: InputEvent) -> void:
	if event.is_action_pressed("MainMenu"):
		get_tree().change_scene_to_file("res://main.tscn")
