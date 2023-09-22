extends Node


signal image_received(Image)


const IMAGE_WIDTH := 1280
const IMAGE_HEIGHT := 720
const IMAGE_SIZE := IMAGE_HEIGHT * IMAGE_WIDTH * 3

#var thr := Thread.new()
#var udp_sender := PacketPeerUDP.new()
var tcp_server := TCPServer.new()
var ffmpeg_tcp: StreamPeerTCP
var buffer := PackedByteArray()

var send_from: int
#var send_to: int


func subprocess():
	var output := []
	var err := OS.execute(
		"ffmpeg",
#		[
#			"-f", "mpegts",
#			"-c:v", "libvpx-vp9",
#			"-i", "udp://127.0.0.1:%s" % send_to,
#			"-f", "rawvideo",
#			"-pix_fmt", "rgb24",
#			"udp://127.0.0.1:%s" % send_from 
#		],
		[
			"-f", "mpegts",
			"-c:v", "libvpx-vp9",
			"-b:v", "1500k",
			"-i", "tcp://127.0.0.1:%s" % send_from,
			"-c", "copy",
			"-map", "0",
			"-f", "segment",
			"-segment_time", "5",
			"-segment_format", "matroska",
			"capture-%03d.mkv",
			
		],
#		output,
#		true
		[],
		false,
		true
	)
	
	if err != OK:
		push_error(output[0])


func _ready() -> void:
	send_from = randi_range(10000, 65000)
#	send_to = randi_range(10000, 65000)
	if tcp_server.listen(send_from, "127.0.0.1") != OK:
		push_error("Failed to start FFMPEG TCP Server")
#	udp_sender.bind(send_from, "127.0.0.1")
#	udp_sender.set_dest_address("127.0.0.1", send_to)
#	thr.start(subprocess)
	OS.create_process(
		"ffmpeg",
#		[
#			"-f", "mpegts",
#			"-c:v", "libvpx-vp9",
#			"-i", "udp://127.0.0.1:%s" % send_to,
#			"-f", "rawvideo",
#			"-pix_fmt", "rgb24",
#			"udp://127.0.0.1:%s" % send_from 
#		],
		[
			"-v", "debug",
#			"-f", "mpegts",
#			"-c:v", "libvpx-vp9",
			"-i", "tcp://127.0.0.1:%s" % send_from,
			"-f", "matroska",
			"out.mkv"
		],
		true
	)


func _process(_delta: float) -> void:
	if ffmpeg_tcp == null and tcp_server.is_connection_available():
		ffmpeg_tcp = tcp_server.take_connection()
#	if udp_sender.get_available_packet_count() == 0:
#		return
#
#	print_debug("receivedee")
#	for _i in range(udp_sender.get_available_packet_count()):
#		buffer.append_array(udp_sender.get_packet())
#		if buffer.size() >= IMAGE_SIZE:
#			var img := Image.create_from_data(IMAGE_WIDTH, IMAGE_HEIGHT, false,Image.FORMAT_RGB8, buffer.slice(0, IMAGE_SIZE))
#			buffer = buffer.slice(IMAGE_SIZE)
#			image_received.emit(img)
#			print_debug("received")


func process_data(data: PackedByteArray):
	if ffmpeg_tcp == null:
		push_error("Failed to send to FFMPEG")
		return
	ffmpeg_tcp.put_data(data)
#	ffmpeg_tcp.disconnect_from_host()
#	ffmpeg_tcp = null


func _exit_tree() -> void:
	if ffmpeg_tcp != null:
		ffmpeg_tcp.disconnect_from_host()
#	thr.wait_to_finish()
