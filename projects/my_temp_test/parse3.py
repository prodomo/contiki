import serial
from collections import OrderedDict
import MySQLdb
from datetime import datetime, timedelta

ser = serial.Serial('/dev/ttyUSB0', 115200)

key_map = OrderedDict({'DATA_LEN':0, 'TIMESTAMP1':1, 'TIMESTANP2':2, 'TIMESYNCTIMESTAMP':3, 'NODE_ID':4, 
           'SEQNO':5, 'HOPS':6, 'LATENCY':7, 'DATA_LEN2':8, 'CLOCK':9, 'TIMESYNCHTIME':10, 
           'TIME_CPU':11, 'TIME_LPM':12, 'TIME_TRANSMIT':13, 'TIME_LISTEN':14, 'BEST_NEIGHBOR':15,
           'BEST_NEIGHBOR_ETX':16, 'RTMETRIC':17, 'NUM_NEIGHBORS':18, 'BEACON_INTERVAL':19, 
           'SENSOR1':20, 'SENSOR2':21, 'SENSOR3':22, 'SENSOR4':23, 'SENSOR5':24, 'SENSOR6':25, 
           'SENSOR7':26, 'SENSOR8':27, 'SENSOR9':28, 'SENSOR10':29})

def upload_to_DB(data):
#  try:
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
      sql = "INSERT INTO itri_MOEA_sensor(sn, mac_addr, \
                ext_temperature, pyranometer, datetime, int_temperature, battery_volt) \
                VALUES ('%d', '%x', '%.2f', '%d', '%s', '%.2f', '%.2f')" %\
                (int(data[key_map['SEQNO']]), int(data[key_map['NODE_ID']]), ext_value,\
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
                VALUES ('%d', '%x', '%.2f', '%d', '%s', '%.2f', '%.2f')" %\
                (int(data[key_map['SEQNO']]), int(data[key_map['NODE_ID']]), ext_value,\
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
        VALUES ('%s', '%d', '%x', '%.2f',\
        '%04x', '%s', '%d', '%d', '%04x', '%d')"\
            %( '0x11', 1, int(data[key_map['NODE_ID']]),\
            int(data[key_map['BEST_NEIGHBOR_ETX']]), int(data[key_map['BEST_NEIGHBOR']]), \
            datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), int(data[key_map['BEST_NEIGHBOR']]), -90)

      rps_sql = "REPLACE INTO itri_topology_current_neighbors(mode, neighborNum, devAddr,PDR,\
        parentAddr, datetime, SN, rank, n1, rssi1)\
        VALUES ('%s', '%d', '%x', '%.2f',\
        '%04x', '%s', '%d', '%d', '%04x', '%d')"\
            %( '0x11', 1, int(data[key_map['NODE_ID']]),\
            int(data[key_map['BEST_NEIGHBOR_ETX']]), int(data[key_map['BEST_NEIGHBOR']]), \
            datetime.now(), int(data[key_map['SEQNO']]), int(data[key_map['RTMETRIC']]), int(data[key_map['BEST_NEIGHBOR']]), -90)

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
#  except:
    print '3.connect to SQL failed !'




while True:
    data = ser.readline()
    if data:
        print(data)
	split_data = data.split(' ',30)
        print 'split_data={0}, s0={1}, s3={2}'.format(split_data, split_data[key_map['DATA_LEN']], split_data[key_map['NODE_ID']])
	upload_to_DB(split_data)
