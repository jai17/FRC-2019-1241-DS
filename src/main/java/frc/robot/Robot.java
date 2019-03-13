/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveToGoal;
import frc.robot.commands.auto.DriveTurn;
import frc.robot.commands.auto.TurnToGoal;
import frc.robot.commands.auto.YeetOffSequence;
import frc.robot.commands.auto.routines.FarRightShipHatch;
import frc.robot.commands.auto.routines.RightRocketCloseLow;
import frc.robot.commands.auto.routines.RightRocketCloseMid;
import frc.robot.loops.CargoLoop;
import frc.robot.loops.CarriageLoop;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.ElevatorLoop;
import frc.robot.loops.HatchLoop;
import frc.robot.loops.Looper;
import frc.robot.loops.RobotPoseTracker;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.Vision;
import frc.robot.util.Logger;
import frc.robot.util.Point;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;

  // Subsystems
  public static Drivetrain drive;
  public static Cargo cargo;
  public static Carriage carriage;
  public static Elevator elevator;
  public static Hatch hatch;
  public static Vision vision;

  // Loops
  public static DrivetrainLoop driveLoop;
  public static CargoLoop cargoLoop;
  public static CarriageLoop carriageLoop;
  public static ElevatorLoop elevatorLoop;
  public static HatchLoop hatchLoop;
  public static RobotState state;
  RobotPoseTracker tracker;

  // Preferences
  Preferences prefs;

  public static double kP_DRIVE, kI_DRIVE, kD_DRIVE, kF_DRIVE;
  public static double kP_DRIVETURN, kI_DRIVETURN, kD_DRIVETURN;
  public static double kP_TURN, kI_TURN, kD_TURN;
  public static double kP_VISION, kI_VISION, kD_VISION;

  // Carriage Preferences
  public static double shooterSpeed;
  public static double shooterSpeedSlow;

  // Vision Preferences
  public static double[] hsvThresholdHue = new double[2];
  public static double[] hsvThresholdSat = new double[2];
  public static double[] hsvThresholdVal = new double[2];

  // Thread visionThread;

  // Logger
  public static Logger logger;

  // Looper
  Looper looper;

  double maxElevatorSpeed = 0;
  double maxPivotSpeed = 0;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // Subsystem Instances
    drive = Drivetrain.getInstance();
    cargo = Cargo.getInstance();
    carriage = Carriage.getInstance();
    elevator = Elevator.getInstance();
    hatch = Hatch.getInstance();
    vision = Vision.getInstance();

    // // Loop Instances
    driveLoop = DrivetrainLoop.getInstance();
    cargoLoop = CargoLoop.getInstance();
    carriageLoop = CarriageLoop.getInstance();
    elevatorLoop = ElevatorLoop.getInstance();
    hatchLoop = HatchLoop.getInstance();
    state = RobotState.getInstance();
    tracker = RobotPoseTracker.getInstance();
    logger = logger.getInstance();

    // Preferences instance
    prefs = Preferences.getInstance();

    // Open Logger File
    logger.openFile("Robot Init");
    looper = new Looper();

    m_chooser.addObject("Drive Test", new DriveDistance(150, 0, 2, 0.8, 1));
    m_chooser.addObject("Drive Turn Test", new DriveTurn(90, 1, 2, 1));
    m_chooser.addObject("FarRightShipHatch", new FarRightShipHatch());
    m_chooser.addObject("Yeet Off Sequence", new YeetOffSequence());
    m_chooser.addObject("TurnToGoal Test", new TurnToGoal(new Point(20.56, 0), 2, 0.5));
    m_chooser.addObject("DriveToGoal Test", new DriveToGoal(new Point(0, 100), 5, 0.8, false));
    m_chooser.addDefault("RightRocketCloseMid", new RightRocketCloseMid());
    m_chooser.addObject("Drive Track Test", new DriveDistance(60, 0, 0.25, 10, true));
    SmartDashboard.putData("Auto modes", m_chooser);

    // Register all loops
    try {
      looper.register(driveLoop);
      looper.register(cargoLoop);
      looper.register(carriageLoop);
      looper.register(elevatorLoop);
      looper.register(hatchLoop);
      looper.register(tracker);
    } catch (Throwable t) {
      System.out.println(t.getMessage());
      System.out.println(t.getStackTrace());
    }

    // //reset sensors
    drive.reset();
    elevator.resetEncoders();
    cargo.resetAngle();
    hatch.resetAngle();
    drive.resetXY();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    logger.close();
    looper.stop();
    maxElevatorSpeed = 0;
    maxPivotSpeed = 0;
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    looper.start();
    logger.openFile("Auton Init");

    // carriageLoop.setClawPos(true);
    drive.reset();
    drive.resetXY();
    drive.resetXY();
    drive.resetXY();
    drive.resetXY();
    carriage.extendCarriage();
    carriage.prisonBreak();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    updateSmartDashboard();
    logger.openFile("Teleop init");
    looper.start();

    // carriageLoop.setClawPos(true);
    // carriageLoop.setSliderPos(true);
    // carriageLoop.setEjectorPos(false);
    driveLoop.selectGear(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();

    // Joystick joy = new Joystick(0);
    // drive.runLeftDrive(joy.getRawAxis(1));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /*
   * Updates the smart dashboard
   */
  public void updateSmartDashboard() {
    // // ROBOT
    SmartDashboard.putNumber("Battery Voltage",
        Math.round(DriverStation.getInstance().getBatteryVoltage() * 100.0) / 100.0); // Battery Voltage

    // DRIVE
    SmartDashboard.putNumber("Robot Yaw", drive.getYaw()); // Gyro Yaw
    SmartDashboard.putNumber("Robot Angle", drive.getAngle()); // Gyro angle
    SmartDashboard.putNumber("Drive Encoder Inches", drive.getAveragePos()); // distance drive has travelled in inches
    SmartDashboard.putNumber("Drive Encoder Right", drive.getRightPos()); // right encoder
    SmartDashboard.putNumber("Drive Encoder Left", drive.getLeftPos()); // left encoder

    double[] xy = drive.getXY();

    SmartDashboard.putNumber("xPos", xy[0]);
    SmartDashboard.putNumber("yPos", xy[1]);

    SmartDashboard.putString("Robot Pose", "X: " + xy[0] + " Y: " + xy[1] + " θ: " + drive.getYaw());
    // 
    SmartDashboard.putNumber("Left Drive Output", drive.getLeftOutput());
    SmartDashboard.putNumber("Right Drive Output", drive.getRightOutput());
    SmartDashboard.putString("Drivetrain State", driveLoop.getControlState().toString());
    SmartDashboard.putBoolean("Drive Gear", driveLoop.getGear());

    kP_DRIVE = prefs.getDouble("kP_DRIVE", 0);
    kI_DRIVE = prefs.getDouble("kI_DRIVE", 0);
    kD_DRIVE = prefs.getDouble("kD_DRIVE", 0);
    kF_DRIVE = prefs.getDouble("kF_DRIVE", 0.12);

    kP_DRIVETURN = prefs.getDouble("kP_DRIVETURN", 0);
    kI_DRIVETURN = prefs.getDouble("kI_DRIVETURN", 0);
    kD_DRIVETURN = prefs.getDouble("kD_DRIVETURN", 0);

    kP_TURN = prefs.getDouble("kP_TURN", 0);
    kI_TURN = prefs.getDouble("kI_TURN", 0);
    kD_TURN = prefs.getDouble("kD_TURN", 0);

    kP_VISION = prefs.getDouble("kP_VISION", 0);
    kI_VISION = prefs.getDouble("kI_VISION", 0);
    kD_VISION = prefs.getDouble("kD_VISION", 0);

    // ELEVATOR
    if (maxElevatorSpeed < elevator.getElevatorSpeed()) {
      maxElevatorSpeed = Math.abs(elevator.getElevatorSpeed());
    }
    SmartDashboard.putString("Elevator State", elevatorLoop.getControlState().toString());
    SmartDashboard.putNumber("Elevator Max Speed", maxElevatorSpeed);
    SmartDashboard.putNumber("Elevator Raw Units", elevator.getElevatorRotations());
    SmartDashboard.putNumber("Elevator Inches", elevator.getElevatorEncoder());

    // Cargo
    SmartDashboard.putNumber("Cargo Pivot Raw", Math.abs(cargo.getRawCargoPivot()));
    SmartDashboard.putNumber("Cargo Pivot Speed", cargo.getRawPivotSpeed());
    SmartDashboard.putString("Cargo State", cargoLoop.getControlState().toString());

    if (maxPivotSpeed < Math.abs(cargo.getRawPivotSpeed())) {
      maxPivotSpeed = Math.abs(cargo.getRawPivotSpeed());
    }

    SmartDashboard.putNumber("Max Pivot Speed", maxPivotSpeed);
    SmartDashboard.putBoolean("Cargo Present Intake", cargo.getOptic());

    // Carriage
    SmartDashboard.putBoolean("Cargo Present Carriage", !carriage.getOptic());
    SmartDashboard.putBoolean("Tray Position", carriageLoop.getSliderPos());
    SmartDashboard.putBoolean("Ejector Position", carriageLoop.getEjectorPos());
    SmartDashboard.putBoolean("Claw Position", carriageLoop.getClawPos());
    SmartDashboard.putNumber("Hatch Detector Right", carriage.getUltrasonicRight());
    SmartDashboard.putNumber("Hatch Detector Left", carriage.getUltrasonicLeft());
    shooterSpeed = prefs.getDouble("shooterSpeed", 1);
    shooterSpeedSlow = prefs.getDouble("shooterSpeedSlow", 0.5);

    // Field Relative Positioning
    SmartDashboard.putString("Position", state.getFieldToRobot().toString());
    SmartDashboard.putNumber("Distance Driven", state.getDistanceDriven());
    SmartDashboard.putNumber("Robot Angle FRP", state.getFieldToRobot().getTheta());

    // VISION
    SmartDashboard.putNumber("X", vision.avg());
    SmartDashboard.putNumber("Degrees To Target", vision.pixelToDegree(vision.avg()));

    hsvThresholdHue[0] = prefs.getDouble("HueMin", 30);
    hsvThresholdHue[1] = prefs.getDouble("HueMax", 180);

    hsvThresholdSat[0] = prefs.getDouble("SatMin", 0);
    hsvThresholdSat[1] = prefs.getDouble("SatMax", 255);

    hsvThresholdVal[0] = prefs.getDouble("ValMin", 0);
    hsvThresholdVal[1] = prefs.getDouble("ValMax", 255);

    SmartDashboard.putString("VISION STATE", vision.getTrackingState().toString());

    //Hatch 
    SmartDashboard.putNumber("Hatch Angle", hatch.getAngle()); 
    SmartDashboard.putNumber("Hatch Speed", hatch.getHatchPivotSpeed()); 


    // Shuffleboard.getTab("Vision").add("H",
    // 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    // Shuffleboard.getTab("Vision").add("S",
    // 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    // Shuffleboard.getTab("Vision").add("V",
    // 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  }
}