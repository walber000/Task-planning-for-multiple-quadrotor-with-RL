import socket
import struct


# Client
ip = '127.0.0.1'
port = 12000
addr = (ip, port)
buff_size = 1024

data = [3.14159,69.69,0.123]
msg = struct.pack('<3f', *data)
while True:
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	client_socket.settimeout(1.0)
	try:
		by = client_socket.sendto(msg, addr)
		print(f'sending... {by}')
	except socket.timeout:
		print('REQUEST TIMED OUT')