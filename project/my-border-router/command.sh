
mosquitto_pub -h fd00::1 -m "2" -t iot-2/cmd/a5aa/fmt/leds
echo "$(date +'%d/%m/%Y %H:%M:%S:%3N')"
exit 0