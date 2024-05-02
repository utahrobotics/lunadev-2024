extends LunabotConn


#class Statistics:
	#var data_received: float
	#var packet_loss: float
	#var delta: float
#
#
#func _run_thr():
	#var last_receive_time := Time.get_ticks_msec()
	#var pinged := false
	#var statistics: Array[Statistics] = []
	#
	#while running:
		#var start_service_time := Time.get_ticks_msec()
		#var result: Array = server.service(200)
		#var delta := (Time.get_ticks_msec() - start_service_time) / 1000.0
		#
		#lunabot_mutex.lock()
		#if lunabot != null and lunabot.is_active():
			#if not pinged and Time.get_ticks_msec() - last_receive_time >= 3000:
				#pinged = true
				#lunabot.send(
					#Channels.IMPORTANT,
					#PackedByteArray([ImportantMessage.PING]),
					#ENetPacketPeer.FLAG_RELIABLE
				#)
			#
			#var stats := Statistics.new()
			#stats.data_received = server.pop_statistic(ENetConnection.HOST_TOTAL_RECEIVED_DATA) + server.pop_statistic(ENetConnection.HOST_TOTAL_SENT_DATA)
			#stats.packet_loss = lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_LOSS) / ENetPacketPeer.PACKET_LOSS_SCALE
			#stats.delta = delta
			#statistics.append(stats)
			#
			#var total_time := 0.0
			#var data_received := 0.0
			#var packet_loss := 0.0
			#
			#for i in range(statistics.size() - 1, -1, -1):
				#stats = statistics[i]
				#total_time += stats.delta
				#data_received += stats.data_received
				#packet_loss += stats.packet_loss
				#if total_time > 2.0:
					#statistics = statistics.slice(i + 1)
					#break
			#
			#data_received /= total_time
			#packet_loss /= total_time
			#
			#call_deferred(
				#"emit_signal",
				#"network_statistics",
				#roundi(data_received),
				#roundi(lunabot.get_statistic(ENetPacketPeer.PEER_ROUND_TRIP_TIME) / 2),
				#roundi(packet_loss),
				#lunabot.get_statistic(ENetPacketPeer.PEER_PACKET_THROTTLE) / ENetPacketPeer.PACKET_THROTTLE_SCALE
			#)
		#lunabot_mutex.unlock()
		#
		#var event_type: ENetConnection.EventType = result[0]
		#var peer: ENetPacketPeer = result[1]
		#var _data: int = result[2]
		#var channel: int = result[3]
		#
		#match event_type:
			#ENetConnection.EventType.EVENT_ERROR:
				#server.destroy()
				#push_error("Server got error, closing!")
				#running = false
				#break
				#
			#ENetConnection.EventType.EVENT_CONNECT:
				#lunabot_mutex.lock()
				#if lunabot != null:
					#push_error("New connection to lunabot even though we are already connected!")
					#lunabot_mutex.unlock()
					#continue
				#lunabot = peer
				#lunabot_mutex.unlock()
				#call_deferred("emit_signal", "connected")
				#
			#ENetConnection.EventType.EVENT_DISCONNECT:
				#lunabot_mutex.lock()
				#if lunabot == null:
					#push_error("Disconnection to lunabot even though we were never connected!")
					#lunabot_mutex.unlock()
					#continue
				#lunabot = null
				#lunabot_mutex.unlock()
				#call_deferred("emit_signal", "disconnected")
				#
			#ENetConnection.EventType.EVENT_RECEIVE:
				#last_receive_time = Time.get_ticks_msec()
				#pinged = false
				#_on_receive(channel)
		#
		#lunabot_mutex.lock()
		#if echo_controls:
			#raw_send_unreliable(Channels.CONTROLS, controls_data)
		#lunabot_mutex.unlock()
	#
	#if lunabot != null and lunabot.is_active():
		#lunabot_mutex.lock()
		#lunabot.peer_disconnect()
		#while true:
			#var result: Array = server.service(0)
			#var event_type: ENetConnection.EventType = result[0]
			#
			#if event_type == ENetConnection.EventType.EVENT_DISCONNECT:
				#break
		#lunabot_mutex.unlock()
	#
	#server.destroy()
