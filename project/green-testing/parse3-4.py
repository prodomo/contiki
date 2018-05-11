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

ser = serial.Serial('/dev/ttyUSB0', 115200)


key_map = OrderedDict({'DATA_LEN':0, 'TIMESTAMP1':1, 'TIMESTANP2':2, 'TIMESYNCTIMESTAMP':3, 'NODE_ID':4, 
           'SEQNO':5, 'HOPS':6, 'LATENCY':7, 'DATA_LEN2':8, 'CLOCK':9, 'TIMESYNCHTIME':10, 
           'TIME_CPU':11, 'TIME_LPM':12, 'TIME_TRANSMIT':13, 'TIME_LISTEN':14, 'BEST_NEIGHBOR':15,
           'BEST_NEIGHBOR_ETX':16, 'RTMETRIC':17, 'NUM_NEIGHBORS':18, 'BEACON_INTERVAL':19, 
           'SENSOR1':20, 'SENSOR2':21, 'SENSOR3':22, 'SENSOR4':23, 'SENSOR5':24, 'SENSOR6':25, 
           'SENSOR7':26, 'SENSOR8':27, 'SENSOR9':28, 'SENSOR10':29})

ack_map = OrderedDict({'DATA_LEN':0, 'NODE_ID':1, 'COMMAND_ID':2 ,'COMMAND_TYPE':3, 'IS_RECEIVED':4})

ask_command_map = OrderedDict({'SW':0, 'COMMAND_TYPE':1, 'MAC':2 ,'COMMAND_ID':3, 'EW':4})


def upload_to_DB(data):
    db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_OpenWSN" )
    cursor = db.cursor()
    sql= "CREATE TABLE IF NOT EXISTS itri_MOEA_current_sensor ( `sn` INT(11), `position` VARCHAR(80), `mac_addr` VARCHAR(45), `led_status` VARCHAR(45), `pyranometer` INT(11), `int_temperature` FLOAT, `ext_temperature` FLOAT, `battery_volt` FLOAT, datetime DATETIME, `ID` INT(11), PRIMARY KEY(`mac_addr`))"

    try:
        cursor.execute(sql)
        db.commit()
    except:
        #db.rollback()
        print '1.connect SQL failed !!'

    

    if(1):
      #mySql cmd
      ext_value = 0.0
      ext_value += float(data[key_map['SENSOR2']]) * 0.007 - 43.9
      int_value = 0.0
      int_value += float(data[key_map['SENSOR3']]) / 1000

      #add by Nancy
      etx = 0.0
      etx += int(data[key_map['BEST_NEIGHBOR_ETX']])
      pdr = 0.0
      pdr += 1.0/((etx/float(data[key_map['HOPS']]))/64)
      rssi = 0.0
      rssi += float(data[key_map['SENSOR6']])-65535-1
      sql = "INSERT INTO itri_MOEA_sensor(sn, mac_addr, \
                ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
                VALUES ('%d', '%s', '%.2f', '%d', '%s', '%.2f', '%.2f')" %\
                (int(data[key_map['SEQNO']]), data[key_map['NODE_ID']], ext_value,\
                 int(data[key_map['SENSOR3']]), \
		 datetime.now(), int_value, 5.0 )
      #rps_sql = "REPLACE INTO itri_MOEA_current_sensor(sn, mac_addr, \
      #          ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
      #          VALUES ('%d', '%s', '%.2f', '%d', '%s', '%.2f', '%.2f')" %\
      #          (1, data[key_map['SENSOR1']], data[key_map['SENSOR2']], data[key_map['SENSOR3']], \
      #          data[key_map['SENSOR4']], data[key_map['SENSOR5']], data[key_map['SENSOR6']])\
      #          (counter, address, temp_real, pyra_raw, currtime, i_temp_real, pyra_real)
      rps_sql = "REPLACE INTO itri_MOEA_current_sensor(sn, mac_addr, \
                ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
                VALUES ('%d', '%s', '%.2f', '%d', '%s', '%.2f', '%.2f')" %\
                (int(data[key_map['SEQNO']]), data[key_map['NODE_ID']], ext_value,\
                 int(data[key_map['SENSOR3']]), \
		 datetime.now(), int_value, 5.0 )
      try:
          # Execute the SQL command
          cursor.execute(sql)
          db.commit()
          cursor.execute(rps_sql)
          # Commit your changes in the database
          print 'D={1}, sql={0}'.format(sql, datetime.now())
          db.commit()
      except:
          # Rollback in case there is any error
          #db.rollback()
          print 'insert DB failed, do not rollback'
      

      ## topology part
      sql = "INSERT INTO itri_topology_neighbors(mode, neighborNum, devAddr,PDR,\
        parentAddr, datetime, SN, rank, n1, rssi1)\
        VALUES ('%s', '%d', '%s', '%.2f',\
        '%04x', '%s', '%d', '%d', '%04x', '%d')"\
            %( '0x11', 1, data[key_map['NODE_ID']],\
            float(pdr), int(data[key_map['BEST_NEIGHBOR']]), \
            datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), int(data[key_map['BEST_NEIGHBOR']]), rssi)

      rps_sql = "REPLACE INTO itri_topology_current_neighbors(mode, neighborNum, devAddr,PDR,\
        parentAddr, datetime, SN, rank, n1, rssi1)\
        VALUES ('%s', '%d', '%s', '%.2f',\
        '%04x', '%s', '%d', '%d', '%04x', '%d')"\
            %( '0x11', 1, data[key_map['NODE_ID']],\
            float(pdr), int(data[key_map['BEST_NEIGHBOR']]), \
            datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), int(data[key_map['BEST_NEIGHBOR']]), rssi)

      try:
          # Execute the SQL command
          cursor.execute(sql)
          db.commit()
          cursor.execute(rps_sql)
          # Commit your changes in the database
          print 'rplace D={1}, sql={0}'.format(rps_sql, datetime.now())
          db.commit()
      except:
          # Rollback in case there is any error
          #db.rollback()
          print 'insert DB failed, do not rollback'


    db.close()
    print '3.connect to SQL failed !'

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
    # print "listen serial"
    data = ser.readline()
    if data:
      print data
      try:
        split_data = data.split()
        if len(split_data)==30 and split_data[key_map['DATA_LEN']] == '30':
            checkIsNewNode(split_data[key_map['NODE_ID']])
            # print 'split_data={0}, s0={1}, s3={2}'.format(split_data, split_data[key_map['DATA_LEN']], split_data[key_map['NODE_ID']])
            # upload_to_DB(split_data)
        elif len(split_data)==5 and split_data[ack_map['DATA_LEN']] == '4':
            print 'get ack'
            revCommand(split_data[ack_map['NODE_ID']], split_data[ack_map['COMMAND_ID']])
        else:
          pass
      except:
        pass

