# Adding the Gazebo/OSRF APT Repository for ROS 2 Harmonic

1. Install required tools:
```
sudo apt update
sudo apt install curl lsb-release gnupg
```
2. Add the OSRF GPG key and repository:
```
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg]
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```
3. Update package lists:
```
sudo apt update
```
> After completing these steps, you can install Gazebo Harmonic–specific ROS 2 packages such as:
> ```
> sudo apt install ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge
> ```
