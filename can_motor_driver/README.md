# can_motor_driver

A simple ROS 2 package for controlling MKS CAN servo motors from a Raspberry Pi or PC, compatible with the Arctos protocol.

## Quick Start

### 1. Push to Public GitHub Repo
- Create a new repo on GitHub (e.g., `can_motor_driver`).
- In your workspace root:
  ```sh
  git init
  git add can_motor_driver
  git commit -m "Initial commit of can_motor_driver"
  git branch -M main
  git remote add origin https://github.com/yourusername/can_motor_driver.git
  git push -u origin main
  ```

### 2. Clone on Raspberry Pi
- On your Raspberry Pi:
  ```sh
  git clone https://github.com/yourusername/can_motor_driver.git
  cd can_motor_driver
  ```

### 3. Install Dependencies
- Install ROS 2, python-can, and colcon tools:
  ```sh
  sudo apt update
  sudo apt install python3-colcon-common-extensions python3-pip
  pip3 install python-can
  # Install ROS 2 dependencies as needed
  ```

### 4. Build and Source the Package
- From your workspace root:
  ```sh
  colcon build --packages-select can_motor_driver
  source install/setup.bash
  ```

### 5. Connect and Bring Up CANable
- Plug in your CANable (or compatible) USB-CAN adapter.
- Bring up the CAN interface (e.g., can0 at 500kbps):
  ```sh
  sudo ip link set can0 up type can bitrate 500000
  ip link show can0
  ```

### 6. Launch the Node and Test
- Launch the node:
  ```sh
  ros2 run can_motor_driver can_motor_node
  # Or with the launch file:
  ros2 launch can_motor_driver launch_can_motor_node.py
  ```
- Watch the logs for CAN send/receive info.

### 7. (Optional) Monitor CAN Traffic
- Use `candump` to see raw CAN traffic:
  ```sh
  sudo apt install can-utils
  candump can0
  ```

---

For more details, see the code and comments in `can_motor_node.py`.
