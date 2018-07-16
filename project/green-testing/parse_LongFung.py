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
order_num = "fakeNumber"
machineNum = 18
start_time = None
mp_time = None
end_time = None
temp_amount = 5


ser = serial.Serial('/dev/ttyUSB0', 115200)

state_map = ['Default', 'SOL_STATE', 'PVT_STATE', 'MP_STATE', 'EOL_STATE', 'START_CLOSE', 'CLOSE', 'START_OPEN', 'OPEN']

current_table_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'SEQNO':2 ,'COMMAND_TYPE':3, 'CURRENT_STATE':4, 'COUNTER':5, 'AMOUNT_COUNTER':6,
  'SUB_STATE1':7, 'SUB_STATE2':8, 'SUB_STATE3':9, 'SUB_STATE4':10, 'SUB_STATE5':11,
  'DISTANCE1':12, 'DISTANCE2':13, 'DISTANCE3':14, 'DISTANCE4':15, 'DISTANCE5':16,
  'TOTAL_V1':17 , 'TOTAL_V2':18, 'TOTAL_V3':19, 'TOTAL_V4':20, 'TOTAL_V5':21,
  'TEMP_V1':22, 'TEMP_V2':23, 'TEMP_V3':24, 'TEMP_V4':25, 'TEMP_V5':26,
  'TOTAL_A1':27 , 'TOTAL_A2':28, 'TOTAL_A3':29, 'TOTAL_A4':30, 'TOTAL_A5':31,
  'TEMP_A1':32, 'TEMP_A2':33, 'TEMP_A3':34, 'TEMP_A4':35, 'TEMP_A5':36})

current_distance_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'SEQNO':2 ,'COMMAND_TYPE':3, 'CURRENT_STATE':4, 'COUNTER':5, 'AMOUNT_COUNTER':6,
  'SUB_STATE1':7, 'SUB_STATE2':8, 'SUB_STATE3':9, 'SUB_STATE4':10, 'SUB_STATE5':11,
  'DISTANCE1':12, 'DISTANCE2':13, 'DISTANCE3':14, 'DISTANCE4':15, 'DISTANCE5':16})

current_v_a_map=OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'SEQNO':2 ,'COMMAND_TYPE':3,
  'TOTAL_V1':4 , 'TOTAL_V2':5, 'TOTAL_V3':6, 'TOTAL_V4':7, 'TOTAL_V5':8,
  'TEMP_V1':9, 'TEMP_V2':10, 'TEMP_V3':11, 'TEMP_V4':12, 'TEMP_V5':13,
  'TOTAL_A1':14, 'TOTAL_A2':15, 'TOTAL_A3':16, 'TOTAL_A4':17, 'TOTAL_A5':18,
  'TEMP_A1':19, 'TEMP_A2':20, 'TEMP_A3':21, 'TEMP_A4':22, 'TEMP_A5':23})


ack_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'COMMAND_ID':2 ,'COMMAND_TYPE':3, 'IS_RECEIVED':4})

ask_command_map = OrderedDict({'SW':0, 'COMMAND_TYPE':1, 'MAC':2 ,'COMMAND_ID':3, 'EW':4})

def update_t3_table(machineNum, name, value):
  
    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()

    t3_table_qury = "SELECT COUNT(*) From T3_table WHERE name='%s' AND machine='%s' "%(name, machineNum)

    t3_table_update = "UPDATE T3_table SET value='%s', datetime='%s' where name='%s' AND machine='%s'" %(value, current_time, name, machineNum)

    t3_table_insert = "INSERT INTO T3_table(machine, name, value, datetime) VALUES('%s', '%s', '%s', '%s')" %(machineNum, name, value, current_time)
    try:
        print "try update_t3_table"
        cursor.execute(t3_table_qury)
        result = cursor.fetchone()

        if result[0] == 0:
          cursor.execute(t3_table_insert)
          print "try t3_table_insert sql success"
        else:
          cursor.execute(t3_table_update)
          print "try t3_table_update sql success"

        db.commit()
    except:
        print "update_t3_table fail"
    db.close()

def update_history_state_to_DB(amount):

    print "update_history_state"

    global start_time
    global pvt_time
    global mp_time
    global end_time
    global temp_amount

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    state_history_sql = "INSERT INTO itri_history_state(machineNum, SOL_STATE, PVT_STATE, MP_STATE, EOL_STATE, PVT_amount, Total_amount)\
     VALUES('%s', '%s', '%s', '%s', '%s', '%d', '%d')" %(machineNum, start_time, pvt_time, mp_time, end_time, temp_amount, amount)

    try:
        print "try update"
        cursor.execute(state_history_sql)
        db.commit()
    except:
        print "update history_state table fail"
    db.close()

def update_state_to_DB(state):

    global start_time

    print "update_state"

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()

    check_table_sql = "SELECT COUNT(*) FROM `itri_current_state` WHERE 1"

    state_insert_sql = "INSERT INTO itri_current_state(SOL_STATE, machineNum, ID) VALUES('%s', '%s', '%d')" %(current_time, machineNum, 1)
    
    state_update_sql = "UPDATE itri_current_state SET %s='%s' WHERE ID=1" %(state_map[state], current_time)

    # t3_update_sql = "INSERT INTO T3_table(name, value, datetime) VALUES('%s', '%s', '%s')" %("A18_status", state_map[state], current_time)
    # t3_update_sql = "INSERT INTO T3_table(machineNum ,name, value, datetime) VALUES('%s', %s', '%s', '%s') ON DUPLICATE KEY UPDATE value='%s', datetime='%s' " %("A18", "status", state_map[state], current_time, state_map[state], current_time)

    try:
        print "check update_state_to_DB"
        cursor.execute(check_table_sql)
        num = cursor.fetchone()
        print num[0]

        # add new row in table
        if num[0]==0:
          print "state_insert_sql!\n"  
          cursor.execute(state_insert_sql)
          print "state_insert_sql success!\n"
        # update exists row
        else:
          cursor.execute(state_update_sql)
          print "state_update_sql success!\n"
        
        # cursor.execute(t3_update_sql)
        db.commit()
        print "t3_update_sql success update_state_to_DB!\n"
    except:
        print "update current_state table fail"
    db.close()
    update_t3_table("A18", "Status", state)

def reset_DB_current_table():

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    state_reset_sql ="UPDATE itri_current_state set SOL_STATE=NULL, PVT_STATE=NULL, MP_STATE=NULL, EOL_STATE=NULL where ID=1"

    try:
        print "try reset"
        cursor.execute(state_reset_sql)
        db.commit()
    except:
        print "update current_state table fail"
    db.close()


def upload_data_to_DB(data):

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()
    print data

    data_sql = "INSERT INTO log(machineNum, seqno, current_state, amount, sub_state1, sub_state2, sub_state3, sub_state4, sub_state5, \
    distance1, distance2, distance3, distance4, distance5, temp_v1, temp_v2, temp_v3, temp_v4, temp_v5, total_v1, total_v2, total_v3, total_v4, total_v5,\
    temp_a1, temp_a2, temp_a3, temp_a4, temp_a5, total_a1, total_a2, total_a3, total_a4, total_a5, datetime)\
     VALUES ('%s', '%d', '%s', '%d', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s','%s', '%s', '%s', '%s', '%s','%s')" \
    %(machineNum, int(data[current_table_map['SEQNO']]), state_map[int(data[current_table_map['CURRENT_STATE']])], int(data[current_table_map['AMOUNT_COUNTER']]),\
    state_map[int(data[current_table_map['SUB_STATE1']])], state_map[int(data[current_table_map['SUB_STATE2']])], state_map[int(data[current_table_map['SUB_STATE3']])], state_map[int(data[current_table_map['SUB_STATE4']])], state_map[int(data[current_table_map['SUB_STATE5']])], \
    data[current_table_map['DISTANCE1']], data[current_table_map['DISTANCE2']], data[current_table_map['DISTANCE3']], data[current_table_map['DISTANCE4']], data[current_table_map['DISTANCE5']], \
    data[current_table_map['TEMP_V1']], data[current_table_map['TEMP_V2']], data[current_table_map['TEMP_V3']], data[current_table_map['TEMP_V4']], data[current_table_map['TEMP_V5']],\
    data[current_table_map['TOTAL_V1']], data[current_table_map['TOTAL_V2']], data[current_table_map['TOTAL_V3']], data[current_table_map['TOTAL_V4']], data[current_table_map['TOTAL_V5']],\
    data[current_table_map['TEMP_A1']], data[current_table_map['TEMP_A2']], data[current_table_map['TEMP_A3']], data[current_table_map['TEMP_A4']], data[current_table_map['TEMP_A5']],\
    data[current_table_map['TOTAL_A1']], data[current_table_map['TOTAL_A2']], data[current_table_map['TOTAL_A3']], data[current_table_map['TOTAL_A4']], data[current_table_map['TOTAL_A5']],current_time)

    # t3_update_sql = "INSERT INTO T3_table(name, value, datetime) VALUES('%s', '%s', '%s') ON DUPLICATE KEY UPDATE value='%s', datetime='%s' " %("A18_amount", data[current_table_map['AMOUNT_COUNTER']], current_time, data[current_table_map['AMOUNT_COUNTER']], current_time)

    try:
        cursor.execute(data_sql)
        print "data_sql success"
        # cursor.execute(t3_update_sql)
        # print "t3_update_sql success"
        db.commit()
    except:
        #db.rollback()
        print 'data insert db fail !!'

    db.close()
    update_t3_table("A18", "Product_Amount", data[current_table_map['AMOUNT_COUNTER']])


def upload_distance_to_DB(data):

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()
    print data

    data_distance_sql = "INSERT INTO log(machineNum, seqno, current_state, amount, sub_state1, sub_state2, sub_state3, sub_state4, sub_state5, \
    distance1, distance2, distance3, distance4, distance5, datetime)\
     VALUES ('%s', '%d', '%s', '%d', '%s', '%s', '%s', '%s', '%s',  '%s', '%s', '%s', '%s', '%s', '%s')" \
    %(machineNum, int(data[current_distance_map['SEQNO']]), state_map[current_state], int(data[current_distance_map['AMOUNT_COUNTER']]),\
    state_map[int(data[current_distance_map['SUB_STATE1']])], state_map[int(data[current_distance_map['SUB_STATE2']])], state_map[int(data[current_distance_map['SUB_STATE3']])], state_map[int(data[current_distance_map['SUB_STATE4']])], state_map[int(data[current_distance_map['SUB_STATE5']])], \
    data[current_distance_map['DISTANCE1']], data[current_distance_map['DISTANCE2']], data[current_distance_map['DISTANCE3']], data[current_distance_map['DISTANCE4']], data[current_distance_map['DISTANCE5']], \
    current_time)
    

    try:
        cursor.execute(data_distance_sql)
        db.commit()
        print "success"
    except:
        #db.rollback()
        print 'data insert db fail !!'

    db.close()

def upload_v_a_to_DB(data):

    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_LongFung" )
    cursor = db.cursor()

    current_time = datetime.now()
    print data

   
    data_v_sql = "INSERT INTO log(machineNum, seqno, current_state, temp_v1, temp_v2, temp_v3, temp_v4, temp_v5, total_v1, total_v2, total_v3, total_v4, total_v5,\
     temp_a1, temp_a2, temp_a3, temp_a4, temp_a5, total_a1, total_a2, total_a3, total_a4, total_a5, datetime)\
     VALUES ('%s', '%d', '%s',  '%s', '%s', '%s', '%s', '%s',  '%s', '%s', '%s', '%s', '%s',  '%s', '%s', '%s', '%s', '%s',  '%s', '%s', '%s', '%s', '%s','%s')" \
    %(machineNum, int(data[current_v_a_map['SEQNO']]), state_map[current_state], \
    data[current_v_a_map['TEMP_V1']], data[current_v_a_map['TEMP_V2']], data[current_v_a_map['TEMP_V3']], data[current_v_a_map['TEMP_V4']], data[current_v_a_map['TEMP_V5']],\
    data[current_v_a_map['TOTAL_V1']], data[current_v_a_map['TOTAL_V2']], data[current_v_a_map['TOTAL_V3']], data[current_v_a_map['TOTAL_V4']], data[current_v_a_map['TOTAL_V5']],\
    data[current_v_a_map['TEMP_A1']], data[current_v_a_map['TEMP_A2']], data[current_v_a_map['TEMP_A3']], data[current_v_a_map['TEMP_A4']], data[current_v_a_map['TEMP_A5']],\
    data[current_v_a_map['TOTAL_A1']], data[current_v_a_map['TOTAL_A2']], data[current_v_a_map['TOTAL_A3']], data[current_v_a_map['TOTAL_A4']], data[current_v_a_map['TOTAL_A5']],current_time)

    try:
        cursor.execute(data_v_sql)
        db.commit()
        print "success"
    except:
        #db.rollback()
        print 'data insert db fail !!'

    db.close()

def reset_value():
    global start_time
    global pvt_time
    global mp_time
    global end_time
    global temp_amount
    global current_state
      
    print "reset value"
    start_time = None
    pvt_time = None
    mp_time = None
    end_time = None
    temp_amount=0
    current_state = 0
    reset_DB_current_table()


def update_time(state):
    global start_time
    global pvt_time
    global mp_time
    global end_time
    global temp_amount

    print "update time"

    if state == 1:
      start_time = datetime.now()
      print start_time
    elif state == 2:
      pvt_time = datetime.now()
      print pvt_time
    elif state == 3:
      mp_time = datetime.now()
      print mp_time
    elif state == 4:
      end_time = datetime.now()
      print end_time


def check_state(state, amount):
    global current_state
    global temp_amount

    if current_state == 0 and state ==4:
      pass
    
    elif current_state != state:
      update_time(state)

      if state > current_state:
        current_state = state
        update_state_to_DB(state)
        if state == 3 : #MP_STATE
          temp_amount = amount
        elif state==4:
          update_history_state_to_DB(amount)
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

        checkIsNewNode(split_data[current_table_map['NODE_ID']])

        if len(split_data)==17 and split_data[current_table_map['DATA_LEN']] == '30': #distance and substate
            check_state(int(split_data[current_table_map['CURRENT_STATE']]), int(split_data[current_table_map['AMOUNT_COUNTER']]))

            upload_distance_to_DB(split_data)
            
        elif len(split_data)==24 and split_data[current_table_map['DATA_LEN']] == '44':
            upload_v_a_to_DB(split_data)
        
        elif len(split_data)==37 and split_data[current_table_map['DATA_LEN']] == '70':
            check_state(int(split_data[current_table_map['CURRENT_STATE']]), int(split_data[current_table_map['AMOUNT_COUNTER']]))

            upload_data_to_DB(split_data)

        elif len(split_data)==4 and split_data[ack_map['DATA_LEN']] == '6':
            print 'get ack'
            revCommand(split_data[ack_map['NODE_ID']], split_data[ack_map['COMMAND_ID']])
        else:
          pass
      except:
        pass

