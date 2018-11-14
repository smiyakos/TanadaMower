hcitool scan

bluetoothctl
agent DisplayYesNo
default-agent
pair 98:D3:32:10:6A:3F
PIN:1234
trust 98:D3:32:10:6A:3F
connect 98:D3:32:10:6A:3F
exit

sudo bluez-simple-agent hci0 98:D3:32:10:6A:3F

sdptool add --channel=22 SP
sdptool browse local

sudo rfcomm listen /dev/rfcomm0 22

&

sudo rfcomm connect 98:D3:32:10:6A:3F 22

⑤受信

cat /dev/rfcomm0

⑥送信

echo "send data" > /dev/rfcomm0
