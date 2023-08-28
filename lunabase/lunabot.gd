extends Node


enum Channels { IMPORTANT, CAMERA, ODOMETRY, CONTROLS }
enum ImportantMessage { ENABLE_CAMERA, DISABLE_CAMERA, STOP_DRIVE, STOP_ARM }

signal connected
signal disconnected
signal camera_frame_received(img: Image)
signal network_statistics(total_bytes: float, ping: int, packet_loss: float, packet_throttle: float)
signal odometry_received(origin: Vector2)
signal something_received

var server: ENetConnection
var running := true
var thread := Thread.new()
var lunabot: ENetPacketPeer
var lunabot_mutex := Mutex.new()


func _ready() -> void:
	server = ENetConnection.new()
	var err := server.create_host_bound("*", 43721, 1, 0, 0, 0)
	if err != OK:
		push_error("Failed to start ENet Server: " + str(err))
		return
#	server.compress(ENetConnection.COMPRESS_ZLIB)
	thread.start(_run_thr)


func is_lunabot_connected() -> bool:
	lunabot_mutex.lock()
	var is_connected = lunabot != null
	lunabot_mutex.unlock()
	return is_connected


func _run_thr():
		while running:
			var result: Array = server.service(500)
			
			lunabot_mutex.lock()
			if lunabot != null:
				call_deferred(
					"emit",
					"network_statistics",
					server.pop_statistic(ENetConnection.HOST_TOTAL_RECEIVED_DATA) + server.pop_statistic(ENetConnection.HOST_TOTAL_SENT_DATA),
					roundi(lunabot.get_statistic(ENetPacketPeer.PEER_ROUND_TRIP_TIME) / 2),
					lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_LOSS) / ENetPacketPeer.PACKET_LOSS_SCALE,
					lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_THROTTLE) / ENetPacketPeer.PACKET_THROTTLE_SCALE
				)
			lunabot_mutex.unlock()
			
			var event_type: ENetConnection.EventType = result[0]
			var peer: ENetPacketPeer = result[1]
			var data: PackedByteArray = result[2]
			var channel: int = result[3]
			
			match event_type:
				ENetConnection.EventType.EVENT_ERROR:
					server.destroy()
					push_error("Server got error, closing!")
					running = false
					break
					
				ENetConnection.EventType.EVENT_NONE:
					continue
					
				ENetConnection.EventType.EVENT_CONNECT:
					lunabot_mutex.lock()
					if lunabot != null:
						push_error("New connection to lunabot even though we are already connected!")
						continue
					lunabot = peer
					lunabot_mutex.unlock()
					call_deferred("emit", "connected")
					
				ENetConnection.EventType.EVENT_DISCONNECT:
					lunabot_mutex.lock()
					if lunabot == null:
						push_error("Disconnection to lunabot even though we were never connected!")
						continue
					lunabot = null
					lunabot_mutex.unlock()
					call_deferred("emit", "disconnected")
					
				ENetConnection.EventType.EVENT_RECEIVE:
					_on_receive(channel, data)


func _on_receive(channel: int, data: PackedByteArray) -> void:
	call_deferred("emit", "something_received")
	match channel:
		Channels.IMPORTANT:
			pass
		
		Channels.CAMERA:
			var frame := Image.new()
			var err := frame.load_webp_from_buffer(data)
			if err != OK:
				push_error("Bad frame: " + str(err))
				return
			call_deferred("emit", "camera_frame_received", frame)
		
		Channels.ODOMETRY:
			var x := data.decode_float(0)
			var y := data.decode_float(4)
			if x == 0.0 or y == 0.0:
				push_error("Invalid odometry origin")
			var origin := Vector2(x, y)
			call_deferred("emit", "odometry_received", origin)
		
		Channels.CONTROLS:
			# We should send from here, not receive
			# Lunabot will receive on this channel
			push_error("Received data on controls channel!")


func raw_send(channel: int, data: PackedByteArray, flags: int) -> void:
	lunabot_mutex.lock()
	if lunabot == null:
		push_warning("Lunabot not connected to yet!")
		lunabot_mutex.unlock()
		await connected
		lunabot_mutex.lock()
	var err := lunabot.send(channel, data, flags)
	lunabot_mutex.unlock()
	if err != OK:
		push_error("Error sending packet: " + str(err))


func raw_send_reliable(channel: int, data: PackedByteArray) -> void:
	raw_send(channel, data, ENetPacketPeer.FLAG_RELIABLE)


func raw_send_unreliable(channel: int, data: PackedByteArray) -> void:
	raw_send(channel, data, ENetPacketPeer.FLAG_UNSEQUENCED | ENetPacketPeer.FLAG_UNRELIABLE_FRAGMENT)


func send_important_msg(msg: ImportantMessage) -> void:
	var data := PackedByteArray([0])
	data.encode_u8(0, msg)
	raw_send_reliable(0, data)


func _exit_tree() -> void:
	running = false
	thread.wait_to_finish()
