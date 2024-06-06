# Robot-KitBot-Offseason

Source code for the 2024 FRC KitBot robot migrated from Java to Python (RobotPy)
(reference: https://github.com/frc2881/2024-Robot-KitBot)

## Installation & Deployment
* Follow the official documentation for installing Python in your development environment (if Python 3.12 is not already installed): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
* Open this project in VSCode and install the Python extension (including extension pack items Pylance and Python Debugger): https://marketplace.visualstudio.com/items?itemName=ms-python.python
* Open the VSCode Command Palette (Ctrl/Command-Shift-P) -> `Python: Create Environment` to create a local Python virtual environment for the project using the installed Python 3.12 interpreter
* Close and reopen the VSCode project to load the newly created Python virtual environment
* Open the VSCode Command Palette (Ctrl/Command-Shift-P) -> `Tasks: Run Task` to access and run the configured user tasks for the project:
  * `RobotPy: Step 1 - Install / Upgrade RobotPy`
  * `RobotPy: Step 2 - Download & Sync RobotPy for roboRIO`
  * `RobotPy: Step 3 - Simulate Robot Code (Optional)`
  * `RobotPy: Step 4 - Deploy Robot Code`  

## Notes
* RobotPy API documentation and guides are available: https://robotpy.readthedocs.io/en/stable/index.html
* This code template uses the same Command framework standard from WPILib while introducing updated patterns that are more appropriate for Python over Java
* Pre-deployment tests are currently disabled as there is a known unresolved issue between the Python REV library and RobotPy where CANSparkMax/Flex instances are not cleaned up properly between test cycles
* For simulation mode, no physics for hardware / software models have been created for this project yet