extends Node


const RECV_FROM := 43721
const USE_ARCHIMEDES := true

enum Channels { IMPORTANT, CAMERA, ODOMETRY, CONTROLS, MAX }
enum ImportantMessage { ENABLE_CAMERA, DISABLE_CAMERA, PING }

signal connected
signal disconnected
signal camera_sdp_received(sdp: String)
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


class Statistics:
	var data_received: float
	var packet_loss: float
	var delta: float


func _run_thr():
	var last_receive_time := Time.get_ticks_msec()
	var pinged := false
	var statistics: Array[Statistics] = []
	
	while running:
		var start_service_time := Time.get_ticks_msec()
		var result: Array = server.service(200)
		var delta := (Time.get_ticks_msec() - start_service_time) / 1000.0
		
		lunabot_mutex.lock()
		if lunabot != null and lunabot.is_active():
			if not pinged and Time.get_ticks_msec() - last_receive_time >= 3000:
				pinged = true
				lunabot.send(
					Channels.IMPORTANT,
					PackedByteArray([ImportantMessage.PING]),
					ENetPacketPeer.FLAG_RELIABLE
				)
			
			var stats := Statistics.new()
			stats.data_received = server.pop_statistic(ENetConnection.HOST_TOTAL_RECEIVED_DATA) + server.pop_statistic(ENetConnection.HOST_TOTAL_SENT_DATA)
			stats.packet_loss = lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_LOSS) / ENetPacketPeer.PACKET_LOSS_SCALE
			stats.delta = delta
			statistics.append(stats)
			
			var total_time := 0.0
			var data_received := 0.0
			var packet_loss := 0.0
			
			for i in range(statistics.size() - 1, -1, -1):
				stats = statistics[i]
				total_time += stats.delta
				data_received += stats.data_received
				packet_loss += stats.packet_loss
				if total_time > 2.0:
					statistics = statistics.slice(i + 1)
					break
			
			data_received /= total_time
			packet_loss /= total_time
			
			call_deferred(
				"emit_signal",
				"network_statistics",
				roundi(data_received),
				roundi(lunabot.get_statistic(ENetPacketPeer.PEER_ROUND_TRIP_TIME) / 2),
				roundi(packet_loss),
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


func _on_receive(channel: int) -> void:
	call_deferred("emit_signal", "something_received")
	lunabot_mutex.lock()
	var data := lunabot.get_packet()
	
	match channel:
		Channels.IMPORTANT:
			pass
		
		Channels.CAMERA:
			call_deferred("emit_signal", "camera_sdp_received", data.get_string_from_utf8())
		
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
	
	print_debug(str(arm_vel) + " " + str(drum_vel))
	lunabot_mutex.lock()
	controls_data.encode_s8(2, roundi(arm_vel * 127))
	if USE_ARCHIMEDES:
		controls_data.encode_s8(3, roundi(drum_vel * 127))
	echo_controls = true
	lunabot_mutex.unlock()


func _exit_tree() -> void:
	running = false
	thread.wait_to_finish()
