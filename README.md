# b3m_driver
ROS hardware interface for B3M servo

```bash
sudo setserial /dev/ttyUSB0 low_latency
sudo su
modprobe ftdi-sio
echo "165C 000a" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```
