import socket, sys

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error, msg:
    sys.stderr.write("[ERROR] %s\n" % msg[1])
    sys.exit(1)

try:
    sock.connect(("127.0.0.1", 54321))
except socket.error, msg:
    sys.stderr.write("[ERROR] %s\n" % msg[1])
    exit(1)


sock.send("SW 02 FFFF 01 01 03 00 01 EW\n")
print sock.recv(1024)
sock.close()