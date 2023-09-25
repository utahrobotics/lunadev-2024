extends Node


const RECV_FROM := 43721
const USE_ARCHIMEDES := false

enum Channels { IMPORTANT, CAMERA, ODOMETRY, CONTROLS, MAX }
enum ImportantMessage { ENABLE_CAMERA, DISABLE_CAMERA, PING }

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
var controls_data: PackedByteArray
var echo_controls := false


func _ready() -> void:
	if USE_ARCHIMEDES:
		controls_data = PackedByteArray([0, 0, 0, 0])
	else:
		controls_data = PackedByteArray([0, 0, 0])
	
	server = ENetConnection.new()
	var err := server.create_host_bound("*", RECV_FROM, 1, 0, 0, 0)
	if err != OK:
		push_error("Failed to start ENet Server: " + str(err))
		return
	thread.start(_run_thr)


func is_lunabot_connected() -> bool:
	lunabot_mutex.lock()
	@warning_ignore("shadowed_variable_base_class")
	var is_connected = lunabot != null
	lunabot_mutex.unlock()
	return is_connected


func _run_thr():
	var last_receive_time := Time.get_ticks_msec()
	var pinged := false
	
	while running:
		var result: Array = server.service(200)
		
		lunabot_mutex.lock()
		if lunabot != null and lunabot.is_active():
			if not pinged and Time.get_ticks_msec() - last_receive_time >= 3000:
				pinged = true
				lunabot.send(
					Channels.IMPORTANT,
					PackedByteArray([ImportantMessage.PING]),
					ENetPacketPeer.FLAG_RELIABLE
				)
			
			call_deferred(
				"emit_signal",
				"network_statistics",
				server.pop_statistic(ENetConnection.HOST_TOTAL_RECEIVED_DATA) + server.pop_statistic(ENetConnection.HOST_TOTAL_SENT_DATA),
				roundi(lunabot.get_statistic(ENetPacketPeer.PEER_ROUND_TRIP_TIME) / 2),
				lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_LOSS) / ENetPacketPeer.PACKET_LOSS_SCALE,
				lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_THROTTLE) / ENetPacketPeer.PACKET_THROTTLE_SCALE
			)
		lunabot_mutex.unlock()
		
		var event_type: ENetConnection.EventType = result[0]
		var peer: ENetPacketPeer = result[1]
		var _data: int = result[2]
		var channel: int = result[3]
		
		match event_type:
			ENetConnection.EventType.EVENT_ERROR:
				server.destroy()
				push_error("Server got error, closing!")
				running = false
				break
				
			ENetConnection.EventType.EVENT_CONNECT:
				lunabot_mutex.lock()
				if lunabot != null:
					push_error("New connection to lunabot even though we are already connected!")
					lunabot_mutex.unlock()
					continue
				lunabot = peer
				lunabot_mutex.unlock()
				call_deferred("emit_signal", "connected")
				
			ENetConnection.EventType.EVENT_DISCONNECT:
				lunabot_mutex.lock()
				if lunabot == null:
					push_error("Disconnection to lunabot even though we were never connected!")
					lunabot_mutex.unlock()
					continue
				lunabot = null
				lunabot_mutex.unlock()
				push_error("Lost connection to lunabot!")
				call_deferred("emit_signal", "disconnected")
				
			ENetConnection.EventType.EVENT_RECEIVE:
				last_receive_time = Time.get_ticks_msec()
				pinged = false
				_on_receive(channel)
		
		lunabot_mutex.lock()
		if echo_controls:
			raw_send_unreliable(Channels.CONTROLS, controls_data)
		lunabot_mutex.unlock()
	
	if lunabot != null and lunabot.is_active():
		lunabot_mutex.lock()
		lunabot.peer_disconnect()
		while true:
			var result: Array = server.service(0)
			var event_type: ENetConnection.EventType = result[0]
			
			if event_type == ENetConnection.EventType.EVENT_DISCONNECT:
				break
		lunabot_mutex.unlock()
	
	server.destroy()


var camera_image_buffers: Array[PackedByteArray]
var camera_image_idx := -1
var collected_fragment_count: int


func _on_receive(channel: int) -> void:
	call_deferred("emit_signal", "something_received")
	lunabot_mutex.lock()
	var data := lunabot.get_packet()
	
	match channel:
		Channels.IMPORTANT:
			pass
		
		Channels.CAMERA:
			var current_camera_image_index := data.decode_u32(0)
			var current_camera_image_fragment_idx := data.decode_u8(4)
			
			if current_camera_image_fragment_idx == 0:
				if current_camera_image_index <= camera_image_idx:
					print_debug("Out of order fragment A")
					return
				
				collected_fragment_count = 1
				camera_image_idx = current_camera_image_index
				var fragment_count := data.decode_u8(5)
				camera_image_buffers.clear()
				camera_image_buffers.append(data.slice(6))
				
				if fragment_count > 1:
					for _i in range(fragment_count - 1):
						camera_image_buffers.append(PackedByteArray())
					return
			
			elif current_camera_image_index != camera_image_idx:
				print_debug("Out of order fragment B")
				return
			
			else:
				if not camera_image_buffers[current_camera_image_fragment_idx].is_empty():
					push_error("Received the same fragment again!")
					return
				
				camera_image_buffers[current_camera_image_fragment_idx] = data.slice(5)
				collected_fragment_count += 1
				
				if collected_fragment_count < camera_image_buffers.size():
					return
			
			data = camera_image_buffers[0]
			
			for i in range(1, camera_image_buffers.size()):
				data.append_array(camera_image_buffers[i])
			
			var img := Image.new()
			var err := img.load_webp_from_buffer(data)
			if err != OK:
				push_error("Error parsing camera frame: %s" % err)
			call_deferred("emit_signal", "camera_frame_received", img)
		
		Channels.ODOMETRY:
			var x := data.decode_float(0)
			var y := data.decode_float(4)
			if x == 0.0 or y == 0.0:
				push_error("Invalid odometry origin")
			var origin := Vector2(x, y)
			call_deferred("emit_signal", "odometry_received", origin)
		
		Channels.CONTROLS:
			if USE_ARCHIMEDES:
				if data.size() != 4:
					push_error("Invalid controls packet")
					return
			elif data.size() != 3:
				push_error("Invalid controls packet")
				return
			
			echo_controls = controls_data != data
	
	lunabot_mutex.unlock()


func raw_send(channel: int, data: PackedByteArray, flags: int) -> void:
	lunabot_mutex.lock()
	if lunabot == null:
		push_warning("Lunabot not connected to yet!")
		lunabot_mutex.unlock()
		return
	var err := lunabot.send(channel, data, flags)
	lunabot_mutex.unlock()
	if err != OK:
		push_error("Error sending packet: " + str(err))


func raw_send_reliable(channel: int, data: PackedByteArray) -> void:
	raw_send(channel, data, ENetPacketPeer.FLAG_RELIABLE)


func raw_send_unreliable(channel: int, data: PackedByteArray) -> void:
	raw_send(channel, data, ENetPacketPeer.FLAG_UNSEQUENCED | ENetPacketPeer.FLAG_UNRELIABLE_FRAGMENT)


func send_important_msg(msg: ImportantMessage) -> void:
	raw_send_reliable(Channels.IMPORTANT, PackedByteArray([msg]))


func send_steering(drive: float, steering: float) -> void:
	if drive > 1:
		drive = 1
		push_warning("Drive greater than 1!")
	if drive < -1:
		drive = -1
		push_warning("Drive lesser than -1!")
	if steering > 1:
		steering = 1
		push_warning("Steering greater than 1!")
	if steering < -1:
		steering = -1
		push_warning("Steering lesser than -1!")
	
	lunabot_mutex.lock()
	controls_data.encode_s8(0, roundi(drive * 127))
	controls_data.encode_s8(1, roundi(steering * 127))
	echo_controls = true
	lunabot_mutex.unlock()


func send_arm_controls(arm_vel: float, drum_vel:=0.0) -> void:
	if arm_vel > 1:
		arm_vel = 1
		push_warning("Arm velocity greater than 1!")
	if arm_vel < -1:
		arm_vel = -1
		push_warning("Arm velocity lesser than -1!")
	if drum_vel > 1:
		drum_vel = 1
		push_warning("Drum velocity greater than 1!")
	if drum_vel < -1:
		drum_vel = -1
		push_warning("Drum velocity lesser than -1!")
	
	lunabot_mutex.lock()
	controls_data.encode_s8(2, roundi(arm_vel * 127))
	if USE_ARCHIMEDES:
		controls_data.encode_s8(3, roundi(drum_vel * 127))
	echo_controls = true
	lunabot_mutex.unlock()


func _exit_tree() -> void:
	running = false
	thread.wait_to_finish()
