# Delta Robot Control System

This project is a Clojure-based control system for a delta robot. It leverages inverse kinematics to calculate motor angles for desired positions, provides motion control functions to home and move the robot, and includes a command driver to send instructions to the robot's hardware via a Raspberry Pi.

## Features

- **Inverse Kinematics**: Computes motor angles required to reach specific (x, y, z) positions.
- **Motion Control**: Homes the robot to its fully retracted position and moves it along predefined paths.
- **Command Serialization**: Serializes motor commands and transmits them to the robot's hardware.
- **Configuration Management**: Allows easy adjustment of robot parameters via an EDN file.

## Requirements

- **Clojure** 1.10 or higher
- **Babashka** (for process management)
- A **Raspberry Pi** with the necessary hardware setup for the delta robot
- SSH access to the Raspberry Pi
- [raspi-stepper-module](https://github.com/billwinkler/raspi-stepper-module): Kernel Module for controlling stepper motors on Raspberry Pi

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/delta-robot-control.git
   cd delta-robot-control
   ```
   
2. **Install dependencies**:
- Ensure Clojure is installed on your system.
- Install Babashka (if not already installed):
  ```bash
  curl -s https://raw.githubusercontent.com/babashka/babashka/master/install | bash
  ```
3. **Set up the Raspberry Pi:**
- Verify the Raspberry Pi is accessible via SSH.
- Update the `raspberry-pi-host` variable in `src/delta_robot/command_driver.clj` to your Raspberry Pi's hostname or IP address (default is `"raspberrypi.local"`).
- Load the stepper driver module
  ``` shell
  cd raspi-stepper-module
  sudo insmod delta-robot.ko
  ```
- Set up kernel module permissions by granting non-root users access to `/dev/delta_robot`.
  ``` shell
  sudo chmod 666 /dev/delta_robot
  ```
- Create a udev rule so the permission change is applied automatically on boot.  Add this line `KERNEL=="delta_robot", MODE="0666"`. And then reload the rules.
  ``` shell
  sudo nano /etc/udev/rules.d/99-delta-robot.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

4. **Configure the robot:**
- Modify `config.edn` to match your delta robot's physical parameters (see Configuration for details).

## Usage
### Homing the Robot
To home the robot (move it to its fully retracted position), use the `home` function. This resets the current angles to the maximum angle defined in the configuration.

``` clojure
(require '[delta-robot.motion :as motion])
(motion/home)
```

**Note:** The homing process includes a 2-second delay (`Thread/sleep 2000`) to allow the motors to complete their movement. Adjust this timing in `src/delta_robot/motion.clj` if your hardware requires a different duration.

### Moving to a Specific Position
To move the robot to a specific (x, y, z) position, compute the required motor commands and send them to the hardware.

``` clojure
(require '[delta-robot.core :as core]
         '[delta-robot.command-driver :as driver])
(let [x 0, y 0, z 275]
  (let [{:keys [commands new-angles]} (core/compute-step-commands x y z)]
    (driver/send-commands commands)
    (reset! core/current-angles new-angles)))
```

**Note:** Depending on your hardware, you may need to add a delay (e.g., `(Thread/sleep 500)`) after sending commands to ensure the movement completes before issuing new instructions.

### Running a Predefined Path
To move the robot along a sequence of positions, use the `move-path` function.

``` clojure
(require '[delta-robot.motion :as motion])
(def moves [[0 0 275] [50 50 275] [-50 50 275] [-50 -50 275] [50 -50 275] [50 50 275] [0 0 275] [0 0 217]])
(motion/move-path moves)
```
## Configuration
The robot's parameters are defined in `config.edn`. Here’s an example configuration:

``` clojure
{:upper-arm-length 130
 :lower-arm-length 325
 :base-radius 100
 :effector-radius 40
 :max-angle 25
 :min-angle -85
 :gear-ratio 9
 :steps-per-rev 1600}
```

- `upper-arm-length`: Length of the arm attached to the motor (in mm).
- `lower-arm-length`: Length of the arm attached to the effector (in mm).
- `base-radius`: Radius of the circle bounding the base triangle (in mm).
- `effector-radius`: Radius of the circle bounding the effector triangle (in mm).
- `max-angle`: Maximum motor angle (fully retracted, in degrees).
- `min-angle`: Minimum motor angle (fully extended, in degrees).
- `gear-ratio`: Gear ratio from the stepper motor to the arm pulley.
- `steps-per-rev`: Number of stepper motor pulses per revolution.

Adjust these values to reflect your delta robot's specifications. Note that arm rotation is measured clockwise. Arms are at 0° when they are in the horizontal position (aligned with the `x-axis`)

## Testing
Unit tests are located in the test directory. To run them, use:

``` shell
bb test:bb
```

These tests current validate the inverse kinematics and motion control logic, using known positions derived from physical measurements and a Fusion 360 model.

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

