import socket, sys 
from thread import *
import Queue
import serial
import time


share_queue = Queue.Queue()
sendSuccess = True
current_Command = ""

def threadWork(client):
    while True:
        msg = client.recv(1024)
        if not msg:
            pass
        else:
            print "Client send: " + msg 
            client.send("You say: " + msg + "\r\n")
            share_queue.put(msg)
            print "share_queue size: ", share_queue.qsize()
            share_queue.task_done()
    client.close()

def sendCommand(command):
    print "send command ->", command
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    ser.write(command)

# def revCommand(command):
#     global sendSuccess
#     if command == current_Command:
#         sendSuccess = True

def checkCommand():
    global sendSuccess
    global current_Command
    print "in checkCommand !\n"
    if sendSuccess == True:
        if share_queue.empty() == False :
            current_Command = share_queue.get()
            sendSuccess = False
            sendCommand(current_Command)
        else:
            print "queue == null!\n"
    else :
        sendCommand(current_Command)
        # time.sleep(0.01)

def createSocketServer(id):
    print "in thread ", id
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error, msg:
        sys.stderr.write("[ERROR] %s\n" % msg[1])
        sys.exit(1)

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', 54321))
    sock.listen(5)

    while True:
        (csock, adr) = sock.accept()
        print "Client Info: ", csock, adr 
        start_new_thread(threadWork, (csock,))

    sock.close()


start_new_thread(createSocketServer, (1, ))
while True:
    # checkCommand()
    time.sleep(10)