import serial
from collections import OrderedDict
import MySQLdb
from datetime import datetime, timedelta
import socket, sys 
from thread import *
import Queue
import time

share_queue = Queue.Queue()
sendSuccess = True
current_Command = ""
hasCommand = False
current_CommandID =0
command_count=0
node_arr=[]
BROADCAST = "FFFF"
current_state = 0
order_num = "21521"
start_time= datetime(2018, 7, 03, 14, 14, 5)
pvt_time = datetime(2018, 7, 03, 14, 15, 5)
mp_time = datetime(2018, 7, 03, 14, 51, 25)
end_time = datetime(2018, 7, 03, 15, 14, 5)
temp_amount = 5


ser = serial.Serial('/dev/ttyUSB0', 115200)

state_map = ['Default', 'SOL_STATE', 'PVT_STATE', 'MP_STATE', 'EOL_STATE', 'START_CLOSE', 'CLOSE', 'START_OPEN', 'OPEN']

current_table_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'SEQNO':2 ,'COMMAND_TYPE':3, 'CURRENT_STATE':4, 'COUNTER':5, 'AMOUNT_COUNTER':6,
  'SUB_STATE1':7, 'SUB_STATE2':8, 'SUB_STATE3':9, 'SUB_STATE4':10, 'SUB_STATE5':11,
  'DISTANCE1':12, 'DISTANCE2':13, 'DISTANCE3':14, 'DISTANCE4':15, 'DISTANCE5':16,
  'TOTAL_V1':17 , 'TOTAL_V2':18, 'TOTAL_V3':19, 'TOTAL_V4':20, 'TOTAL_V5':21,
  'TEMP_V1':22, 'TEMP_V2':23, 'TEMP_V3':24, 'TEMP_V4':25, 'TEMP_V5':26})

ack_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'COMMAND_ID':2 ,'COMMAND_TYPE':3, 'IS_RECEIVED':4})

ask_command_map = OrderedDict({'SW':0, 'COMMAND_TYPE':1, 'MAC':2 ,'COMMAND_ID':3, 'EW':4})


def update_history_state(amount):

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    state_history_sql = "INSERT INTO itri_history_state(orderNum, SOL_STATE, PVT_STATE, MP_STATE, EOL_STATE, PVT_amount, Total_amount)\
     VALUES('%s', '%s', '%s', '%s', '%s', '%d', '%d')" %(order_num, start_time, pvt_time, mp_time, end_time, temp_amount, amount)

    try:
        print "try update"
        cursor.execute(state_history_sql)
        db.commit()
    except:
        print "update history_state table fail"
    db.close()

def update_state():

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    state_update_sql = "INSERT INTO itri_current_state(SOL_STATE, PVT_STATE, MP_STATE, EOL_STATE, orderNum, ID) VALUES('%s', '%s', '%s', '%s', '%s', '%d') \
    ON DUPLICATE KEY UPDATE SOL_STATE='%s', PVT_STATE='%s', MP_STATE='%s', EOL_STATE='%s', orderNum='%s'" \
    %(start_time, pvt_time, mp_time, end_time, order_num, 1, start_time, pvt_time, mp_time, end_time, order_num) 

    try:
        print "try update"
        cursor.execute(state_update_sql)
        db.commit()
    except:
        print "update current_state table fail"
    db.close()


def upload_to_DB(data):

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()
    print data

   
    data_sql = "INSERT INTO log(orderNum, seqno, current_state, amount, sub_state1, sub_state2, sub_state3, sub_state4, sub_state5, \
    distance1, distance2, distance3, distance4, distance5, temp_v1, temp_v2, temp_v3, temp_v4, temp_v5, total_v1, total_v2, total_v3, total_v4, total_v5, datetime)\
     VALUES ('%s', '%d', '%s', '%d', '%s', '%s', '%s', '%s', '%s', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d', '%d','%s')" \
    %(order_num, int(data[current_table_map['SEQNO']]), state_map[current_state], int(data[current_table_map['AMOUNT_COUNTER']]),\
    state_map[int(data[current_table_map['SUB_STATE1']])], state_map[int(data[current_table_map['SUB_STATE2']])], state_map[int(data[current_table_map['SUB_STATE3']])], state_map[int(data[current_table_map['SUB_STATE4']])], state_map[int(data[current_table_map['SUB_STATE5']])], \
    int(data[current_table_map['DISTANCE1']]), int(data[current_table_map['DISTANCE2']]), int(data[current_table_map['DISTANCE3']]), int(data[current_table_map['DISTANCE4']]), int(data[current_table_map['DISTANCE5']]), \
    int(data[current_table_map['TEMP_V1']]), int(data[current_table_map['TEMP_V2']]), int(data[current_table_map['TEMP_V3']]), int(data[current_table_map['TEMP_V4']]), int(data[current_table_map['TEMP_V5']]),\
    int(data[current_table_map['TOTAL_V1']]), int(data[current_table_map['TOTAL_V2']]), int(data[current_table_map['TOTAL_V3']]), int(data[current_table_map['TOTAL_V4']]), int(data[current_table_map['TOTAL_V5']]),\
    current_time)

    try:
        cursor.execute(data_sql)
        db.commit()
        print "success"
    except:
        #db.rollback()
        print 'data insert db fail !!'

    db.close()

def reset_value():
    start_time = ''
    pvt_time = ''
    mp_time = ''
    end_time = ''
    temp_amount=0


def check_state(data, amount):
    if current_state != data:
      if data> current_state:
        current_state = data
        update_state()
      elif data == 3 : #MP_STATE
        temp_amount = amount
      elif data==0:
        current_state=data
        update_history_state(amount)
        reset_value()

def threadWork(client):
    global hasCommand
    global share_queue
    global command_count
    while True:
        msg = client.recv(1024)
        if not msg:
            pass
        else:
            print "Client send: " + msg 
            client.send("You say: " + msg + "\r\n")
            command_count=command_count+1

            temp_command = msg.split()
            if(temp_command[0] == "SW" and temp_command[len(temp_command)-1] == "EW"):
              temp_command[2]= temp_command[2].upper()
              temp_command[3] = command_count
              print temp_command
              temp = " ".join(str(x) for x in temp_command)+"\n"
              print temp
              share_queue.put(temp)
              print "share_queue size: ", share_queue.qsize()
              hasCommand = True
            share_queue.task_done()
    client.close()

def sendCommand(command):
    # print "send command ->", command
    if command != "None":
      ser = serial.Serial('/dev/ttyUSB0', 115200)
      ser.write(command)

def checkIsContinueSend(nodeId):
    global node_arr
    global current_Command
    is_allSend = True

    print "in check ContinueSend"
    split_command = current_Command.split()
    mac = split_command[ask_command_map['MAC']].lower()

    if mac == BROADCAST or mac == BROADCAST.lower():
      for i in range(len(node_arr)):
        if(node_arr[i][0].lower() == nodeId.lower()):
          node_arr[i][1]=1
        if(node_arr[i][1] == 0):
          is_allSend = False
    elif mac.lower() != nodeId.lower(): 
      is_allSend = False

    return is_allSend

def checkIsNewNode(nodeId):
    global node_arr

    for i in range(len(node_arr)):
      if(node_arr[i][0] == nodeId):
        return
    new_node= [nodeId, 0]
    node_arr.append(new_node)
    print node_arr


def revCommand(nodeId, commandId):
    global sendSuccess
    global current_Command
    global hasCommand
    global current_CommandID
    # print "hope command to get :", recword
    print "current_CommandID = {0}, command={1}".format(current_CommandID, commandId)

    if int(current_CommandID) ==  int(commandId):
      print "get the same command"
      if checkIsContinueSend(nodeId) == True:
        sendSuccess = True
        for i in range(len(node_arr)):
          node_arr[i][1]=0
        if share_queue.empty():
          print "no command to send"
          hasCommand = False
          current_Command=""


def checkCommand(id):
    print "in thread ", id
    global sendSuccess
    global current_Command
    global hasCommand
    global share_queue
    global current_CommandID
    print "in checkCommand !\n"
    while  True:
      if sendSuccess == True:
        time.sleep(10)
        if share_queue.empty() == False :
          print "to get new command to send"
          current_Command = share_queue.get()
          sendSuccess = False
          split_Command = current_Command.split()
          current_CommandID = split_Command[ask_command_map['COMMAND_ID']]
          print "current_CommandID={0}\n".format(current_CommandID)
          sendCommand(current_Command)
        else:
          # print "queue == null!\n"
          hasCommand = False
          current_Command=""
          # time.sleep(10)
      else :
        # for i in range (0,10):
        sendCommand(current_Command)
        time.sleep(1)

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
start_new_thread(checkCommand, (2, ))
while True:
    print "listen serial"
    data = ser.readline()
    if data:
      print data
      try:
        split_data = data.split()
        print "len ",format(len(split_data))

        if len(split_data)==27 and split_data[current_table_map['DATA_LEN']] == '50':
            print "split_data "
            checkIsNewNode(split_data[key_map['NODE_ID']])
      #       # print 'split_data={0}, s0={1}, s3={2}'.format(split_data, split_data[key_map['DATA_LEN']], split_data[key_map['NODE_ID']])
            upload_to_DB(split_data)
            check_state(split_data[current_table_map['CURRENT_STATE']], split_data[current_table_map['AMOUNT_COUNTER']])

      #   elif len(split_data)==5 and split_data[ack_map['DATA_LEN']] == '4':
      #       print 'get ack'
      #       revCommand(split_data[ack_map['NODE_ID']], split_data[ack_map['COMMAND_ID']])
        # else:
          # pass
      except:
        pass

