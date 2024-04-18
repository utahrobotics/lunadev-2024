extends Node


signal log_received(String)
signal portal_disconnected
signal portal_connected

var server_addr := "192.168.0.100"
var ws: WebSocketPeer
var logs: Array[String] = []
var connecting := false
var connected := false


func connect_to_lunaportal():
	if ws != null:
		disconnect_from_lunaportal()
	connecting = true
	connected = false
	ws = WebSocketPeer.new()
	var err := ws.connect_to_url("ws://" + server_addr)
	if err != OK:
		push_error("Failed to connect: %s" % err)


func disconnect_from_lunaportal():
	if ws == null:
		return
	connecting = false
	ws.close()


func _process(_delta: float) -> void:
	if ws != null:
		ws.poll()
		for _i in range(ws.get_available_packet_count()):
			var data := ws.get_packet().get_string_from_utf8()
			log_received.emit(data)
			logs.push_back(data)
		if connecting and ws.get_ready_state() == WebSocketPeer.STATE_OPEN:
			connecting = false
			connected = true
			portal_connected.emit()
		if ws.get_ready_state() == WebSocketPeer.STATE_CLOSED:
			ws = null
			connected = false
			portal_disconnected.emit()


func _exit_tree() -> void:
	disconnect_from_lunaportal()
	
	while connected:
		_process(0)
		OS.delay_msec(50)
