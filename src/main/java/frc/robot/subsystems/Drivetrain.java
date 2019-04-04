package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.ElectricalConstants;
import frc.robot.NumberConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.PID.PIDController;
import frc.robot.commands.drive.TankDrive;
import frc.robot.loops.Loop;
import frc.robot.util.Point;

import java.security.interfaces.DSAKey;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

  // Drive instance
  private static Drivetrain mInstance;
  private Cargo cargo;
  private Elevator elevator;
  // Robot State
  private RobotState robotState;
  // Talon Control Mode constant
  private ControlMode talonControlMode = ControlMode.PercentOutput;

  // drive pose
  private double prevLVel = 0, prevRVel = 0;
  private Point drivePoint;
  private double prevAvgDist = 0;
  private double xPos, yPos;

  // current limits
  private int smartCurrentLimit = 60;

  /* Spark Drive Speed Controllers */
  public CANSparkMax leftMaster;
  private CANSparkMax leftSlave1;
  // private CANSparkMax leftSlave2;

  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave1;
  // private CANSparkMax rightSlave2;

  /* Encoders on the drive */

  private boolean leftEncoderConnected = false;
  private boolean rightEncoderConnected = false;

  /** Gyro on the drive */
  AHRS gyro;

  /* PID */
  private PIDController drivePID;
  private PIDController turnPID;
  PIDController gyroPID;
  private PIDController lockPID;

  /* Solenoid Shifter */
  private DoubleSolenoid shifterSolenoid;
  private boolean isLow = false;

  /* Lifter Cylinder */
  private Solenoid lifterSolenoid;

  // Create a single instance of the drivetrain
  public static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance = new Drivetrain();
      return mInstance;
    } else
      return mInstance;
  }

  // Drivetrain setup
  public Drivetrain() {

    cargo = Cargo.getInstance();
    elevator = Elevator.getInstance();

    /* SparkMax Speed Controllers */
    leftMaster = new CANSparkMax(ElectricalConstants.LEFT_DRIVE_FRONT, MotorType.kBrushless);
    // leftMaster.setInverted(true);
    leftSlave1 = new CANSparkMax(ElectricalConstants.LEFT_DRIVE_MIDDLE, MotorType.kBrushless);
    // leftSlave2 = new CANSparkMax(ElectricalConstants.LEFT_DRIVE_BACK,
    // MotorType.kBrushless);
    // leftMaster.restoreFactoryDefaults();
    // leftSlave1.restoreFactoryDefaults();
    // leftSlave2.restoreFactoryDefaults();
    leftSlave1.follow(leftMaster);
    // leftSlave2.follow(leftMaster);

    rightMaster = new CANSparkMax(ElectricalConstants.RIGHT_DRIVE_FRONT, MotorType.kBrushless);
    // rightMaster.setInverted(true);
    rightSlave1 = new CANSparkMax(ElectricalConstants.RIGHT_DRIVE_MIDDLE, MotorType.kBrushless);
    // rightSlave2 = new CANSparkMax(ElectricalConstants.RIGHT_DRIVE_BACK,
    // MotorType.kBrushless);
    // rightMaster.restoreFactoryDefaults();
    // rightSlave1.restoreFactoryDefaults();
    // rightSlave2.restoreFactoryDefaults();
    rightSlave1.follow(rightMaster);
    // rightSlave2.follow(rightMaster);

    // Initialize PID controllers
    drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
    turnPID = new PIDController(NumberConstants.pTurn, NumberConstants.iTurn, NumberConstants.dTurn);
    lockPID = new PIDController(NumberConstants.pLock, NumberConstants.iLock, NumberConstants.dLock);
    gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);

    // Initialize shifter solenoid
    shifterSolenoid = new DoubleSolenoid(ElectricalConstants.SHIFTER_SOLENOID_HIGH,
        ElectricalConstants.SHIFTER_SOLENOID_LOW);
    lifterSolenoid = new Solenoid(ElectricalConstants.LIFTER_SOLENOID);

    // Initialize navX gyro
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("ERROR navX: " + ex.getMessage(), true);
    }

    // reset encoders, gyro, shift high
    reset();
    // shiftLow();

    this.setRightBrakeMode();
    this.setLeftBrakeMode();

    leftMaster.setSmartCurrentLimit(smartCurrentLimit);
    leftSlave1.setSmartCurrentLimit(smartCurrentLimit);
    rightMaster.setSmartCurrentLimit(smartCurrentLimit);
    rightSlave1.setSmartCurrentLimit(smartCurrentLimit);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  public void reset() {
    resetDriveEncoders();
    resetGyro();
  }

  public double getRightPos() {
    return (cargo.getDriveRightInches());
  }

  public double getLeftPos() {
    return (elevator.getDriveLeftInches());
  }

  public double getAveragePos() {
    return (getRightPos() + getLeftPos()) / 2;
  }

  public void resetDriveEncoders() {
    cargo.resetRightDrive();
    elevator.resetLeftDrive();
  }

  // drive methods

  public void setLeftBrakeMode() {
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave1.setIdleMode(IdleMode.kBrake);
  }

  public void setRightBrakeMode() {
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave1.setIdleMode(IdleMode.kBrake);
  }

  public void setLeftCoastMode() {
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave1.setIdleMode(IdleMode.kCoast);
  }

  public void setRightCoastMode() {
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave1.setIdleMode(IdleMode.kCoast);
  }

  public void setLeftrampRate(double rate) {
    leftMaster.setOpenLoopRampRate(rate);
  }

  public void setRightrampRate(double rate) {
    rightMaster.setOpenLoopRampRate(rate);
  }

  public void runLeftDrive(double speed) {
    leftMaster.set(speed);
  }

  public void runRightDrive(double speed) {
    rightMaster.set(speed);
  }

  public double getLeftBusVoltage() {
    return leftMaster.getBusVoltage();
  }

  public double getRightBusVoltage() {
    return rightMaster.getBusVoltage();
  }

  public double getLeftMotorTemperature() {
    return leftMaster.getMotorTemperature();
  }

  public double getRightMotorTemperature() {
    return rightMaster.getMotorTemperature();
  }

  public double getLeftOutput() {
    return leftMaster.getAppliedOutput();
  }

  public double getRightOutput() {
    return rightMaster.getAppliedOutput();
  }

  public double getAverageRaw() {
    return (elevator.getDriveLeftRaw() + cargo.getDriveRightRaw()) / 2;
  }

  public double getAverageOutput() {
    return (getRightOutput() + getLeftOutput()) / 2;
  }

  public double getLeftVelocityInchesPerSec() {
    return elevator.getLeftVelocityInchesPerSec();
  }

  public double getRightVelocityInchesPerSec() {
    return cargo.getRightVelocityInchesPerSec();
  }

  public double getAverageVelInchesPerSec() {
    return 0;
  }

  // gyro methods
  // get absolute angle
  public double getAngle() {
    return gyro.getAngle();
  }

  // get angle relative to where robot is turned on
  public double getYaw() {
    return gyro.getYaw();
  }

  // reset gyro
  public void resetGyro() {
    gyro.zeroYaw();
  }

  // solenoid methods
  // get whether solenoid is in high or low
  public DoubleSolenoid.Value getIslow() {
    return shifterSolenoid.get();
  }

  // shift to low gear
  public void shiftLow() {
    shifterSolenoid.set(DoubleSolenoid.Value.kForward);
    isLow = true;
  }

  // shift to high gear
  public void shiftHigh() {
    shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
    isLow = false;
  }

  // get if drive is in low gear
  public boolean getIsLow() {
    return isLow;
  }

  // extend lifter
  public void extendLifter() {
    lifterSolenoid.set(true);
  }

  //retract lifter
  public void retractLifter() {
    lifterSolenoid.set(false);
  }

  // PID methods
  // drive PID
  public void drivePID(double distSetpoint, double angleSetpoint, double speed, double epsilon) {
    drivePID.changePIDGains(Robot.kP_DRIVE, Robot.kI_DRIVE, Robot.kD_DRIVE);
    turnPID.changePIDGains(Robot.kP_TURN - 0.005, Robot.kI_TURN, Robot.kD_TURN - 0.);

    double driveOut = drivePID.calcPIDDrive(distSetpoint, getAveragePos(), 1);
    double angleOut = turnPID.calcPIDDrive(angleSetpoint, getYaw(), 1);
    // driveOut = Math.min(driveOut, 0.25);

    // if(driveOut > 0) {
    // runLeftDrive(driveOut + angleOut + Robot.kF_DRIVE);
    // runRightDrive(-driveOut - angleOut + Robot.kF_DRIVE);
    // } else if (driveOut < 0) {
    // runLeftDrive(driveOut + angleOut - Robot.kF_DRIVE);
    // runRightDrive(-driveOut - angleOut - Robot.kF_DRIVE);
    // }

    runLeftDrive((driveOut + angleOut) * speed);
    runRightDrive((-driveOut + angleOut) * speed);

    System.out.println(this.toString() + ":drivePID RUNNING: tracking: " + angleSetpoint);
  }

  // drive PID constrained to top speed
  public void regulatedDrivePID(double distSetpoint, double angleSetpoint, double epsilon, double topSpeed,
    boolean track, boolean high) {
    
    //shift gears, change constants
    if (high) {
      this.shiftHigh();
      drivePID.changePIDGains(Robot.kP_DRIVE * 0.1, Robot.kI_DRIVE, Robot.kD_DRIVE * 5);
    } else {
      this.shiftLow();
      if (!track){
      drivePID.changePIDGains(Robot.kP_DRIVE, Robot.kI_DRIVE, Robot.kD_DRIVE);
      } else {
        drivePID.changePIDGains(Robot.kP_DRIVE * 0.7, Robot.kI_DRIVE, Robot.kD_DRIVE * 1);
      }
    }

    //change to use angles
    if (!track) {
      turnPID.changePIDGains(Robot.kP_DRIVETURN, Robot.kI_DRIVETURN, Robot.kD_DRIVETURN);
    } else {
      //turnPID.changePIDGains(Robot.kP_DRIVETURN * 0.75, Robot.kI_DRIVETURN, Robot.kD_DRIVETURN * 1);

      turnPID.changePIDGains(Robot.kP_VISION * 0.85, Robot.kI_VISION, Robot.kD_VISION * 1);
    }

    double currentVal = getAngle();

    double driveOut = drivePID.calcPIDDrive(distSetpoint, getAveragePos(), epsilon);
    // limit driving PID output to top speed
    if (driveOut > 0) // driving forward
      driveOut = Math.min(driveOut, topSpeed);
    else if (driveOut < 0) // driving backward
      driveOut = Math.max(driveOut, -topSpeed);

    double angleOut = turnPID.calcPIDDrive(angleSetpoint, currentVal, epsilon);

    // if (relative) {
    //   if ((angleSetpoint < -90 || angleSetpoint > 90) && currentVal < 0) {
    //     angleOut = -angleOut;
    //   }
    // }

    double leftOut = driveOut + angleOut;
    double rightOut = -driveOut + angleOut;

    // drive robot
    runLeftDrive(leftOut);
    runRightDrive(rightOut);
  }

  // turn PID
  public void turnPID(double angleSetpoint, double speed, double epsilon) {
    turnPID.changePIDGains(Robot.kP_TURN, Robot.kI_TURN, Robot.kD_TURN);
    double angleOut = turnPID.calcPIDDrive(angleSetpoint, getYaw(), epsilon) * speed;

    runLeftDrive(angleOut);
    runRightDrive(angleOut);
  }

  // track turn PID
  public void trackTurnPID(double angleSetpoint, double speed, double epsilon, double stick, double maxOutput) {
    turnPID.changePIDGains(Robot.kP_VISION, Robot.kI_VISION, Robot.kD_VISION);
    double angleOut = turnPID.calcPIDDrive(angleSetpoint, getAngle(), epsilon) * speed;

    // Max speed clamping to 50%
    if (angleOut > 0) {
      angleOut = Math.min(angleOut, maxOutput);
    } else if (angleOut < 0) {
      angleOut = Math.max(angleOut, -maxOutput);
    }

    runLeftDrive(angleOut - stick);
    runRightDrive(angleOut + stick);
  }

  // turn PID constrained to top speed for P2P
  public void regulatedTurnPID(double angleSetpoint, double epsilon, double topSpeed, boolean track) {
    if (!track) {
      turnPID.changePIDGains(Robot.kP_TURN, Robot.kI_TURN, Robot.kD_TURN);
    } else {
      turnPID.changePIDGains(Robot.kP_VISION, Robot.kI_VISION, Robot.kD_VISION);
    }
    this.shiftLow(); 

    double currentVal = getAngle();

    double angleOut = turnPID.calcPIDDrive(angleSetpoint, currentVal, epsilon);
    
    // limit turning PID output to top speed
    if (angleOut > 0) // driving forward
      angleOut = Math.min(angleOut, topSpeed);
    else if (angleOut < 0) // driving backward
      angleOut = Math.max(angleOut, -topSpeed);

    runLeftDrive(angleOut);
    runRightDrive(angleOut);
  }

  public boolean drivePIDDone() {
    return drivePID.isDone();
  }

  public void resetDrivePID() {
    drivePID.resetPID();
  }

  public boolean gyroPIDDone() {
    return gyroPID.isDone();
  }

  public void resetGyroPID() {
    gyroPID.resetPID();
  }

  public void changeDriveGains(double p, double i, double d) {
    drivePID.changePIDGains(p, i, d);
  }

  public void changeGyroGains(double p, double i, double d) {
    gyroPID.changePIDGains(p, i, d);
  }

  /************ Motion Magic Methods *******/
  /*
   * public void runDriveMotionMagic(double setpoint) {
   * rightMaster.set(ControlMode.MotionMagic, setpoint);
   * leftMaster.set(ControlMode.MotionMagic, setpoint); }
   * 
   * public double getMotionMagicError() { return
   * (rightMaster.getClosedLoopError(0) + leftMaster.getClosedLoopError())/2; }
   * 
   * public void magicMotionSetpoint(double setpoint, int cruiseVelocity, double
   * secsToMaxSpeed) { rightMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
   * rightMaster.configMotionAcceleration((int) (NumberConstants.DRIVE_MAX_SPEED /
   * secsToMaxSpeed), 0);
   * 
   * leftMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
   * leftMaster.configMotionAcceleration((int) (NumberConstants.DRIVE_MAX_SPEED /
   * secsToMaxSpeed), 0);
   * 
   * runDriveMotionMagic(setpoint * -ElectricalConstants.DRIVE_TO_INCHES); }
   * 
   * public double getLeftOutput() { return leftMaster.getMotorOutputPercent(); }
   * 
   * public double getRightOutput() { return rightMaster.getMotorOutputPercent();
   * }
   */

  /*********** Ramp Rates ****************/
  /*
   * public void setLeftRampRate(double rampRate) {
   * leftMaster.set(ControlMode.Velocity, rampRate); }
   * 
   * public void setRightRampRate(double rampRate) {
   * rightMaster.set(ControlMode.Velocity, rampRate); }
   */

  // reset encoders and gyro

  // set brake mode
  /*
   * public void setBrakeMode() {
   * leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
   * rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
   * }
   * 
   * //set coast mode public void setCoastMode() {
   * leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
   * rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
   * }
   */

  // POSE
  // set xPos
  public void setXPos(double newXPos) {
    drivePoint.setXpos(newXPos);
  }

  // set yPos
  public void setYPos(double newYPos) {
    drivePoint.setYpos(newYPos);
  }

  // reset x and y
  public void resetXY() {
    xPos = 0;
    yPos = 0;
    prevAvgDist = 0;
  }

  // set x and y to a value
  public void setXY(double x, double y) {
    xPos = x;
    yPos = y;
  }

  // get field-relative position of drive
  // within a single timestamp, the robot will not move out of quadrants 1 or 2
  // (won't turn 90+ degrees in 10 ms)
  public double[] getXY() {
    double deltaDist = getAveragePos() - prevAvgDist;
    double currentAngle = getYaw();

    xPos += deltaDist * Math.sin(Math.toRadians(currentAngle));
    yPos += deltaDist * Math.cos(Math.toRadians(currentAngle));

    prevAvgDist = getAveragePos();

    return new double[] { xPos, yPos };
  }

  // get field relative position of drive as a point
  public Point getXYPoint() {
    return new Point(xPos, yPos);
  }

  // get previous average distance
  public double getPrevAvgDist() {
    return prevAvgDist;
  }
}