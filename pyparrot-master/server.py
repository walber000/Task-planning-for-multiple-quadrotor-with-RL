import socket
import struct


# Server
# ip = '127.0.0.1'
ip = '192.168.0.41'
port = 12000
buff_size = 1024
addr = (ip, port)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('', port))

while True:
	try:
		data, server = server_socket.recvfrom(buff_size)
		data = struct.unpack('<4f', data)
		print(data)
	except KeyboardInterrupt:
		print('Interrupt by user...')
		break
