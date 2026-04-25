# Installing Mosquitto MQTT Broker on Ubuntu (Linux)

This guide shows how to install and test the Mosquitto MQTT broker on Ubuntu (desktop, server, or WSL), and then optionally install MQTT Explorer (AMD/Intel only).
---

## 1. Prerequisites

- A running Ubuntu system (20.04, 22.04, or later).
- A user with `sudo` privileges.
- Internet access to reach Ubuntu and Mosquitto repositories.

Check Ubuntu version:

```bash
lsb_release -a
```


---

## 2. Update System Packages

Refresh package lists:

```bash
sudo apt update
```

(Optional but recommended) Upgrade existing packages:

```bash
sudo apt upgrade -y
```


---

## 3. (Optional) Add Mosquitto PPA for Newer Version

Ubuntu’s repo may not have the latest Mosquitto; you can add the official PPA to get newer releases.

```bash
sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa -y
sudo apt update
```

If you are fine with Ubuntu’s default version, you can skip this step and use the next section directly.

---

## 4. Install Mosquitto Broker and Clients

Install the broker and CLI client tools:

```bash
sudo apt install -y mosquitto mosquitto-clients
```


Enable and start Mosquitto as a service:

```bash
sudo systemctl enable mosquitto      # start at boot
sudo systemctl start mosquitto       # start immediately
```

Check status:

```bash
systemctl status mosquitto
```

You should see `active (running)` in the output.

---

## 5. Quick Local Test (Publish/Subscribe)

Open two terminals.

### Terminal 1 – Subscriber

```bash
mosquitto_sub -t "test/topic"
```

This subscribes to the `test/topic` topic and waits for messages.

### Terminal 2 – Publisher

```bash
mosquitto_pub -t "test/topic" -m "hello from mosquitto"
```

You should see `hello from mosquitto` appear in Terminal 1, confirming that the broker and clients work locally.

---

## 6. Basic Configuration

Mosquitto config directory on Ubuntu:

- Main config: `/etc/mosquitto/mosquitto.conf`
- Additional configs: `/etc/mosquitto/conf.d/*.conf`

Create a simple config for local development:

```bash
sudo nano /etc/mosquitto/conf.d/local.conf
```

Example content (insecure, for local tests only):

```conf
listener 1883
allow_anonymous true
```


Save and restart Mosquitto:

```bash
sudo systemctl restart mosquitto
```

Now clients can connect without authentication on port 1883.

---

## 7. Enabling Username/Password Authentication (Recommended)

For real use (beyond localhost), you should disable anonymous access and require a username/password.

### 7.1 Create a Password File

Create (or overwrite) a password file and add a user:

```bash
sudo mosquitto_passwd -c /etc/mosquitto/passwd myuser
```

You will be prompted to enter a password.

To add another user later (without `-c`):

```bash
sudo mosquitto_passwd /etc/mosquitto/passwd anotheruser
```


### 7.2 Update Configuration

Edit or create an auth-enabled config:

```bash
sudo nano /etc/mosquitto/conf.d/auth.conf
```

Example:

```conf
listener 1883
allow_anonymous false
password_file /etc/mosquitto/passwd
```


Restart the service:

```bash
sudo systemctl restart mosquitto
```


### 7.3 Test Authenticated Connection

Subscriber:

```bash
mosquitto_sub -h localhost -p 1883 -u myuser -P 'my_password' -t "secure/topic"
```

Publisher:

```bash
mosquitto_pub -h localhost -p 1883 -u myuser -P 'my_password' -t "secure/topic" -m "secured message"
```


If authentication is working, the subscriber will receive `secured message`.

---

## 8. Optional: Secure with TLS (High-Level Only)

For production over the internet, you should also enable TLS (SSL) with certificates (e.g., via Let’s Encrypt) so credentials and payloads are encrypted.

This typically involves:

- Obtaining certificates (e.g., `/etc/letsencrypt/live/your-domain/fullchain.pem` and `privkey.pem`).
- Adding `cafile`, `certfile`, and `keyfile` to your Mosquitto listener config.
- Restarting Mosquitto.

(Full TLS configuration is beyond this brief install guide.)

---

## 9. Optional: Install MQTT Explorer on Ubuntu (AMD/Intel Only)

MQTT Explorer is a GUI client for inspecting topics and payloads; it runs on x86_64/AMD64 architectures (Intel/AMD CPUs).

### Option A – Snap Package (Ubuntu)

Ensure `snapd` is installed:

```bash
sudo apt update
sudo apt install -y snapd
```

Log out and back in (or reboot) so snap paths are set, then install MQTT Explorer:

```bash
sudo snap install mqtt-explorer
```

You can then launch it from your application menu or via:

```bash
mqtt-explorer
```


### Option B – AppImage (Portable)

If you prefer a portable binary, use the AppImage.

1. Download the MQTT Explorer AppImage for Linux (AMD64) from its page.
2. Make it executable:

```bash
chmod +x MQTT-Explorer-*.AppImage
```

3. Run it:

```bash
./MQTT-Explorer-*.AppImage
```


Within MQTT Explorer, configure:

- Host: `localhost` (or your broker IP).
- Port: `1883` (or TLS port).
- Username/Password if you enabled authentication. 
---

You now have Mosquitto running on Ubuntu with optional GUI tooling via MQTT Explorer (on AMD/Intel). 

```


