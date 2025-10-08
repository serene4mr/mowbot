# Mosquitto MQTT Broker on Ubuntu: Installation & Verification

**Install Mosquitto and its client tools, enable and start the service, then confirm it’s handling messages with a simple publish–subscribe test:**

1. **Update package lists**  
   ```bash
   sudo apt update
   ```

2. **Install broker and client utilities**  
   ```bash
   sudo apt install mosquitto mosquitto-clients -y
   ```

3. **Enable on boot and start the broker**  
   ```bash
   sudo systemctl enable mosquitto
   sudo systemctl start mosquitto
   ```

4. **Check that Mosquitto is running**  
   ```bash
   sudo systemctl status mosquitto
   ```

5. **Subscribe to a test topic** (open in one terminal)  
   ```bash
   mosquitto_sub -h localhost -t test/topic -v
   ```

6. **Publish a message to that topic** (in another terminal)  
   ```bash
   mosquitto_pub -h localhost -t test/topic -m "Hello MQTT"
   ```

If the subscriber terminal displays:
```
test/topic Hello MQTT
```
then the broker is correctly installed and operational.
