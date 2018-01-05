my-border-route project copyed from /example/ipv6/rpl-border-route
my-testing project copyed from /example/cc2538-common/mqtt-demo
green-testing project copyed from green's github

my-border-route project is for root and using with tunslip6.
my-testing project is for mote.
my-border-route should be used with my-testing project.

my-testing project use subscribe and publish function to get command and send packet in TCP/IP

for update to db, use paho-mqtt
1.install paho-mqtt 
	$pip install paho-mqtt
2.To obtain the full code, including examples and tests, you can clone the git repository:
	$git clone https://github.com/eclipse/paho.mqtt.python
3.Once you have the code, it can be installed from your repository as well:
	$cd paho.mqtt.python
	$python setup.py install