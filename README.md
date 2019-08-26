# FRC-2019-1241-DS

Team 1241's 2019 FRC robot code for [Zapdos](https://www.thebluealliance.com/team/1241/2019). Zapdos's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Code Highlights
* Path following with Bezier Curves and PID Controller

    To control autonomous driving.

* Roborio based Computer Vision for target detection
  
  Utilizes OpenCV librariers and a USB camera
    
## Package Functions
- [`com.team1241.frc2019`](src/com/team1241/frc2019)

    Contains the robot's central functions and holds a class with all numerical constants used throughout the code
