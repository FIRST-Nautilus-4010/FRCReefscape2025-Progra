# FRCReefscape2025

Welcome to the official repository for Robotics Team 4010's FRC robot for the 2025 season.

---

## Overview

This project contains all the code for our robot, designed to compete in the FIRST Robotics Competition (FRC) 2025. The robot is programmed using Java and leverages the WPILib library, along with several vendor libraries for hardware integration.

The game rules and objectives are detailed in the [2025 FRC Game Manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf).

---

## About the Team

We are Team 4010, a group of students and mentors passionate about robotics, technology, and innovation. Our mission is to design, build, and program competitive robots while learning and sharing knowledge in STEM fields.

---

## Project Structure

The repository is organized as follows:

- **src/**: Main source code for the robot.
  - **main/java/frc/robot/**: Root package for robot code.
    - **commands/**: Contains command classes that define robot actions and behaviors, such as driving with joysticks.
    - **subsystems/**: Contains subsystem classes representing hardware components (e.g., swerve drive, elevator, coprocessor).
      - **swerve/**: Swerve drive implementation (see note below).
      - **Elevator/**: Elevator mechanism control.
    - **utils/**: Utility classes, constants, and helper functions.
  - **main/deploy/**: Files to be deployed to the RoboRIO (e.g., configuration files).
- **vendordeps/**: Vendor dependency JSON files for third-party hardware libraries (CTRE Phoenix, REV, Studica, etc.).
- **build.gradle**: Gradle build configuration.
- **.wpilib/**: WPILib-specific configuration files.
- **README.md**: This documentation file.

---

## How the Code Works

### Robot Architecture

The code follows the Command-Based programming paradigm recommended by WPILib. This design separates robot logic into **subsystems** (representing hardware) and **commands** (representing actions or behaviors).

#### Subsystems

- **Swerve Drive (`subsystems/swerve/`)**:  
  Implements a swerve drive system, allowing the robot to move in any direction and rotate independently.  
  **Note:** The swerve subsystem is still under development and does not yet follow a fully modular or ideal structure. Some logic may be tightly coupled or lack abstraction, and further refactoring is planned.

  - `Swerve.java`: Main class managing the four swerve modules, odometry, and gyro integration.
  - `SwerveModule.java`: Represents an individual swerve module, handling drive and turning motors, encoders, and PID control.

- **Elevator (`subsystems/Elevator/`)**:  
  Controls the elevator mechanism, including motor management, sensor feedback, and PID-based position/angle control.

  - `ElevatorSubsystem.java`: Main subsystem class, manages state and periodic control logic.
  - `ElevatorIO.java`: Handles direct motor control and configuration.
  - `ElevatorController.java`: Contains PID controllers for height and angle.
  - `ElevatorSensors.java`: Reads sensor values for feedback.

- **Coprocessor (`subsystems/Coprocessor.java` and `utils/CoprocessorClient.java`)**:  
  Manages communication with an external coprocessor for advanced tasks such as pathfinding. Uses a custom protocol over sockets to send/receive messages and coordinates.

#### Commands

- **SwerveDriveJoystick (`commands/SwerveDriveJoystick.java`)**:  
  Reads joystick input and translates it into swerve drive commands, applying deadzones, rate limiting, and field-relative control.

- **Other Commands**:  
  Additional commands can be added to automate sequences or respond to operator input.

#### Utilities

- **Constants (`utils/Constants.java`)**:  
  Central location for all robot constants, such as dimensions, PID gains, and kinematics.

- **Other Utilities**:  
  Helper classes for math, communication, and configuration.

---

## Key Features

- **Swerve Drive**:  
  Four independently controlled modules for omnidirectional movement and rotation. Uses WPILib's kinematics and odometry classes for precise control and pose estimation.

- **Elevator Mechanism**:  
  PID-controlled for both height and angle, supporting manual and automatic modes.

- **Coprocessor Integration**:  
  Supports advanced features like pathfinding by communicating with an external processor.

- **Simulation Support**:  
  Configured for WPILib simulation, allowing code testing without hardware.

- **Vendor Hardware Support**:  
  Integrates CTRE Phoenix (TalonFX, CANcoder), REV (SparkMax), and Studica hardware.

---

## How to Build and Deploy

1. **Clone the repository:**
   ```sh
   git clone https://github.com/your-org/FRCReefscape2025.git
   cd FRCReefscape2025
   ```

2. **Build the project:**
   ```sh
   ./gradlew build
   ```

3. **Deploy to the RoboRIO:**
   ```sh
   ./gradlew deploy
   ```

4. **Simulate (optional):**
   ```sh
   ./gradlew simulateJava
   ```

---

## How to Contribute

1. Clone the repository.
2. Create a new branch for your feature:
   ```sh
   git checkout -b feature/new-feature
   ```
3. Make your changes and commit them:
   ```sh
   git commit -m 'Add new feature'
   ```
4. Push your branch:
   ```sh
   git push origin feature/new-feature
   ```
5. Open a Pull Request.

---

## Notes and Future Improvements

- The swerve subsystem is still being refactored and does not yet follow a fully modular or ideal structure. Expect changes and improvements in future commits.
- Additional documentation and code comments are being added to improve maintainability.
- More automated tests and simulation scenarios will be included as the season progresses.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Contact

For more information, visit our website or contact us through our social media channels.

Thank you for your interest in robotics team 4010!