import paho.mqtt.client as mqtt
import MySQLdb
from datetime import datetime, timedelta
from collections import OrderedDict

# MQTT Settings 
# MQTT_Broker = "fd00::1"
MQTT_Broker = ""
MQTT_Port = 1883
Keep_Alive_Interval = 60
MQTT_Topic = "#"
# MQTT_Topic = "iot-2/evt/status/fmt/json"

key_map = OrderedDict({'MAC':0, 'SEQ':1, 'PARENT':2, 'RSSI':3, 'RANK':4, 
           'ETX':5, 'HOPS':6, 'TEMP':7})
# , 'VDD':8

#Subscribe to all Sensors at Base Topic
def upload_to_DB(data):
	db = MySQLdb.connect("127.0.0.1","root","sakimaru","ITRI_OpenWSN" )
	cursor = db.cursor()
	sql= "CREATE TABLE IF NOT EXISTS itri_MOEA_current_sensor ( `sn` INT(11), `position` VARCHAR(80), `mac_addr` VARCHAR(45), `led_status` VARCHAR(45), `pyranometer` INT(11), `int_temperature` FLOAT, `ext_temperature` FLOAT, `battery_volt` FLOAT, datetime DATETIME, `ID` INT(11), PRIMARY KEY(`mac_addr`))"
	
	etx = 0.0
	etx += int(data[key_map['ETX']])
	hops=0.0
	hops += int(data[key_map['HOPS']])
	if(etx !=0 and hops !=0):
		pdr = 0.0
		pdr += 1.0/(etx/(64*hops))
	else:
		pdr=0.0
	parent = data[key_map['PARENT']].split(':', 6)
	try:
		cursor.execute(sql)
		db.commit()
	except:
		print '1.connect SQL failed !!'

	if(1):

		sensorsql="INSERT INTO itri_MOEA_sensor(sn, mac_addr, ext_temperature, pyranometer, datetime, int_temperature, battery_volt)\
		VALUES('%d', '%s', '%.2f', '%d', '%s', '%.2f', '%.2f')" %(int(data[key_map['SEQ']]), data[key_map['MAC']], 0, 0, datetime.now(), (int(data[key_map['TEMP']])), 5.0)

		cur_sensorsql="REPLACE INTO itri_MOEA_current_sensor(sn, mac_addr, ext_temperature, pyranometer, datetime, int_temperature, battery_volt)\
		VALUES('%d', '%s', '%.2f', '%d', '%s', '%.2f', '%.2f')" %(int(data[key_map['SEQ']]), data[key_map['MAC']], 0, 0, datetime.now(), (int(data[key_map['TEMP']])), 5.0)


		nbrsql = "INSERT INTO itri_topology_neighbors(mode, neighborNum, devAddr,PDR,\
        parentAddr, datetime, SN, rank, n1, rssi1)\
        VALUES ('%s', '%d', '%s', '%.2f',\
        '%s', '%s', '%d', '%d', '%s', '%d')"\
            %( '0x11', 1, data[key_map['MAC']],\
            float(pdr), parent[5], \
            datetime.now(), int(data[key_map['SEQ']]), int(data[key_map['RANK']]), parent[5], int(data[key_map['RSSI']]))
		
		cur_nbrsql = "REPLACE INTO itri_topology_current_neighbors(mode, neighborNum, devAddr,PDR,\
        parentAddr, datetime, SN, rank, n1, rssi1)\
        VALUES ('%s', '%d', '%s', '%.2f',\
        '%s', '%s', '%d', '%d', '%s', '%d')"\
            %( '0x11', 1, data[key_map['MAC']],\
            float(pdr), parent[5], \
            datetime.now(), int(data[key_map['SEQ']]), int(data[key_map['RANK']]), parent[5], int(data[key_map['RSSI']]))

		try:
			cursor.execute(sensorsql)
			db.commit()
			cursor.execute(cur_sensorsql)
			db.commit()
			cursor.execute(nbrsql)
			db.commit()
			cursor.execute(cur_nbrsql)
			db.commit()
			print 'insert to db'
		except:
			print 'insert DB failed, do not rollback'
	db.close()

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_Topic)

def on_message(client, userdata, msg):
	print "Data: " + msg.payload
	# split_data = msg.payload.split(' ',8)
	# upload_to_DB(split_data)


client = mqtt.Client()

# Assign event callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connect
client.connect(MQTT_Broker, 1883, 60)

# Continue the network loop
client.loop_forever()

