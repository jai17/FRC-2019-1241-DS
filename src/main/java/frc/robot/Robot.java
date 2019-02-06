/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.CargoLoop;
import frc.robot.loops.CarriageLoop;
import frc.robot.loops.DrivetrainLoop;
import frc.robot.loops.ElevatorLoop;
import frc.robot.loops.HatchLoop;
import frc.robot.loops.Looper;
import frc.robot.loops.RobotPoseTracker;
import frc.robot.loops.VisionLoop;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hatch;
import frc.robot.util.Logger;

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

  // Loops
  public static DrivetrainLoop driveLoop;
  public static CargoLoop cargoLoop;
  public static CarriageLoop carriageLoop;
  public static ElevatorLoop elevatorLoop;
  public static HatchLoop hatchLoop;
  public static VisionLoop visionLoop;
  public RobotState state;
  RobotPoseTracker tracker;

  // Preferences
  Preferences prefs;

  // Vision Preferences
  public static double[] hsvThresholdHue = new double[2];
  public static double[] hsvThresholdSat = new double[2];
  public static double[] hsvThresholdVal = new double[2];

  Thread visionThread;

  // Logger
  public static Logger logger;

  // Looper
  Looper looper;

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

    // Loop Instances
    driveLoop = DrivetrainLoop.getInstance();
    cargoLoop = CargoLoop.getInstance();
    carriageLoop = CarriageLoop.getInstance();
    elevatorLoop = ElevatorLoop.getInstance();
    hatchLoop = HatchLoop.getInstance();
    state = RobotState.getInstance();
    tracker = RobotPoseTracker.getInstance();
    logger = logger.getInstance();
    visionLoop = VisionLoop.getInstance();

    // Preferences instance
    prefs = Preferences.getInstance();

    // Open Logger File
    logger.openFile("Robot Init");
    looper = new Looper();

    // m_chooser.addDefault("Default Auto", new ExampleCommand());
    // chooser.addObject("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    // Register all loops
    try {
      looper.register(driveLoop);
      looper.register(cargoLoop);
      looper.register(carriageLoop);
      looper.register(elevatorLoop);
      looper.register(hatchLoop);
      looper.register(visionLoop);
      looper.register(tracker);
    } catch (Throwable t) {
      System.out.println(t.getMessage());
      System.out.println(t.getStackTrace());
    }

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
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
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

    logger.openFile("Teleop init");
    looper.start();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void updateSmartDashboard() {
    // ROBOT
    SmartDashboard.putNumber("Battery Voltage",Math.round(DriverStation.getInstance().getBatteryVoltage() * 100.0) / 100.0); // Battery Voltage

    // DRIVE
    SmartDashboard.putNumber("Robot Angle", drive.getYaw()); // Gyro Yaw
    SmartDashboard.putNumber("Drive Encoders", drive.getAvgPos()); // distance drive has travelled in inches

    //Field Relative Positioning
    SmartDashboard.putString("Position", state.getFieldToRobot().toString()); 
    SmartDashboard.putNumber("Distance Driven", state.getDistanceDriven()); 
    SmartDashboard.putNumber("Robot Angle FRP", state.getFieldToRobot().getTheta()); 
    SmartDashboard.putNumber("RobotVelocity", state.getVelocityDelta().getR()); 

    // VISION
    SmartDashboard.putNumber("X", visionLoop.avg());
    SmartDashboard.putNumber("Degrees To Target", visionLoop.pixelToDegree(visionLoop.avg()));

    hsvThresholdHue[0] = prefs.getDouble("HueMin", 50);
    hsvThresholdHue[1] = prefs.getDouble("HueMax", 180);

    hsvThresholdSat[0] = prefs.getDouble("SatMin", 0);
    hsvThresholdSat[1] = prefs.getDouble("SatMax", 255);

    hsvThresholdVal[0] = prefs.getDouble("ValMin", 238);
    hsvThresholdVal[1] = prefs.getDouble("ValMax", 255);

  }
}