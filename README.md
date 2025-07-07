# Delta Robot Control System

This project is a Clojure-based control system for a delta robot. It leverages inverse kinematics to calculate motor angles for desired positions, provides motion control functions to home and move the robot, and includes a command driver to send instructions to the robot's hardware via a Raspberry Pi.

## Features

- **Inverse Kinematics**: Computes motor angles required to reach specific (x, y, z) positions.
- **Motion Control**: Homes the robot to its fully retracted position and moves it along predefined paths.
- **Command Generation**: Generates synchronized pulse waveforms for stepper motors and executes them on a Raspberry Pi using `pigpiod`.
- **Configuration Management**: Allows easy adjustment of robot parameters via an EDN file.

## Requirements

- **Clojure** 1.10 or higher
- **Babashka** (for process management)
- A **Raspberry Pi** with `pigpiod` installed and running.

## Installation

1. **Clone the repository onto your Raspberry Pi**:
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
- Install `pigpiod`:
  ```bash
  sudo apt-get update
  sudo apt-get install pigpio
  ```
- Enable and start the `pigpiod` daemon:
  ```bash
  sudo systemctl enable pigpiod
  sudo systemctl start pigpiod
  ```

4. **Configure the robot:**
- Modify `config.edn` to match your delta robot's physical parameters and GPIO pin assignments.

## Usage
### Homing the Robot
To home the robot (move it to its fully retracted position), use the `home` function. This resets the current angles to the maximum angle defined in the configuration.

``` clojure
(require '[delta-robot.motion :as motion])
(motion/home)
```
**WARNING:** The `home` function does not use limit switches. Ensure your robot has physical stops to prevent damage during the homing sequence.

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
 :steps-per-rev 1600
 ;; pigpiod configuration
 :pulse-width-us 100
 :min-frequency 500.0
 :max-frequency 2000.0
 :acceleration-pulses 150
 :deceleration-pulses 150
 :pulse-overhead-ns 113909
 :gpio-pins {:motor0 {:step 17 :dir 27}
             :motor1 {:step 18 :dir 23}
             :motor2 {:step 19 :dir 25}
             :limit-switches [22 24 26]}}
```

- `upper-arm-length`: Length of the arm attached to the motor (in mm).
- `lower-arm-length`: Length of the arm attached to the effector (in mm).
- `base-radius`: Radius of the circle bounding the base triangle (in mm).
- `effector-radius`: Radius of the circle bounding the effector triangle (in mm).
- `max-angle`: Maximum motor angle (fully retracted, in degrees).
- `min-angle`: Minimum motor angle (fully extended, in degrees).
- `gear-ratio`: Gear ratio from the stepper motor to the arm pulley.
- `steps-per-rev`: Number of stepper motor pulses per revolution.
- `pulse-width-us`: The duration of the stepper pulse in microseconds.
- `min-frequency`: The starting frequency for acceleration.
- `max-frequency`: The target frequency for the stepper motors.
- `acceleration-pulses`: The number of pulses to use for acceleration.
- `deceleration-pulses`: The number of pulses to use for deceleration.
- `pulse-overhead-ns`: A fudge factor for estimating motion duration.
- `gpio-pins`: The GPIO pins for the stepper motors and limit switches.

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

