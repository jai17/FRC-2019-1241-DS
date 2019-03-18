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
  private int smartCurrentLimit = 55;

  /* Drive speed controlers */
  // public TalonSRX leftMaster;
  // private VictorSPX leftSlave1;
  // private VictorSPX leftSlave2;

  // private TalonSRX rightMaster;
  // private VictorSPX rightSlave1;
  // private VictorSPX rightSlave2;

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

    /*
     * //Initialize Talons //left master leftMaster = new
     * TalonSRX(ElectricalConstants.LEFT_DRIVE_FRONT);
     * leftMaster.configSelectedFeedbackSensor(FeedbackDevice.
     * CTRE_MagEncoder_Relative, 0, 0); leftMaster.setInverted(true);
     * leftMaster.setSensorPhase(true);
     * leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * 
     * //left slaves leftSlave1 = new
     * VictorSPX(ElectricalConstants.LEFT_DRIVE_BACK);
     * leftSlave1.set(ControlMode.Follower, ElectricalConstants.LEFT_DRIVE_FRONT);
     * leftSlave1.follow(leftMaster); leftSlave2 = new
     * VictorSPX(ElectricalConstants.LEFT_DRIVE_MIDDLE);
     * leftSlave2.set(ControlMode.Follower, ElectricalConstants.LEFT_DRIVE_FRONT);
     * leftSlave2.follow(leftMaster);
     * leftSlave2.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * leftSlave1.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * 
     * 
     * //right masters rightMaster = new
     * TalonSRX(ElectricalConstants.RIGHT_DRIVE_FRONT);
     * rightMaster.configSelectedFeedbackSensor(FeedbackDevice.
     * CTRE_MagEncoder_Relative, 0, 0); rightMaster.setInverted(false);
     * rightMaster.setSensorPhase(false);
     * rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * 
     * //right slaves rightSlave2 = new
     * VictorSPX(ElectricalConstants.RIGHT_DRIVE_MIDDLE);
     * rightSlave2.set(ControlMode.Follower, ElectricalConstants.RIGHT_DRIVE_FRONT);
     * rightSlave2.follow(rightMaster); rightSlave1 = new
     * VictorSPX(ElectricalConstants.RIGHT_DRIVE_BACK);
     * rightSlave1.set(ControlMode.Follower, ElectricalConstants.RIGHT_DRIVE_FRONT);
     * rightSlave1.follow(rightMaster);
     * rightSlave1.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * rightSlave2.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
     * 
     * //For Motion Magic set up rightMaster.selectProfileSlot(0, 0);
     * rightMaster.configMotionAcceleration(2500, 0);
     * rightMaster.configMotionCruiseVelocity(2500, 0); rightMaster.config_kF(0,
     * NumberConstants.fTalonDrive, 10); rightMaster.config_kP(0,
     * NumberConstants.pTalonDrive, 0); rightMaster.config_kI(0,
     * NumberConstants.iTalonDrive, 0); rightMaster.config_kD(0,
     * NumberConstants.dTalonDrive, 0);
     * 
     * //For Motion Magic set up leftMaster.selectProfileSlot(0, 0);
     * leftMaster.configMotionAcceleration(2500, 0);
     * leftMaster.configMotionCruiseVelocity(2500, 0); leftMaster.config_kF(0,
     * NumberConstants.fTalonDrive, 10); leftMaster.config_kP(0,
     * NumberConstants.pTalonDrive, 0); leftMaster.config_kI(0,
     * NumberConstants.iTalonDrive, 0); leftMaster.config_kD(0,
     * NumberConstants.dTalonDrive, 0); leftMaster.setInverted(false);\
     */

    // Initialize PID controllers
    drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
    turnPID = new PIDController(NumberConstants.pTurn, NumberConstants.iTurn, NumberConstants.dTurn);
    lockPID = new PIDController(NumberConstants.pLock, NumberConstants.iLock, NumberConstants.dLock);
    gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro);

    // Initialize shifter solenoid
    shifterSolenoid = new DoubleSolenoid(ElectricalConstants.SHIFTER_SOLENOID_HIGH,
        ElectricalConstants.SHIFTER_SOLENOID_LOW);

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
  /*
   * TalonSRX Methods public void runLeftDrive(double input) {
   * leftMaster.set(ControlMode.PercentOutput, input); }
   * 
   * public void runRightDrive(double input) {
   * rightMaster.set(ControlMode.PercentOutput, input); }
   * 
   * //encoder methods //get left encoder public double getLeftPos() { return
   * leftMaster.getSelectedSensorPosition(0) / ElectricalConstants.TICKS_PER_INCH;
   * }
   * 
   * //get right encoder public double getRightPos() { return
   * rightMaster.getSelectedSensorPosition(0) /
   * ElectricalConstants.TICKS_PER_INCH; }
   * 
   * // average encoder value public double getAveragePos() { return (getLeftPos()
   * + getRightPos()) / 2; }
   * 
   * //average raw encoder ticks public double getAverageRaw() { return
   * ((leftMaster.getSelectedSensorPosition(0) +
   * rightMaster.getSelectedSensorPosition(0)) / 2); }
   * 
   * public double getLeftSpeed() { return
   * leftMaster.getSelectedSensorVelocity(0); }
   * 
   * public double getRightSpeed() { return
   * rightMaster.getSelectedSensorVelocity(0); }
   * 
   * public double getLeftVelocityInchesPerSec() { return ((getLeftSpeed() /
   * ElectricalConstants.DRIVE_TO_INCHES) * 10); }
   * 
   * public double getRightVelocityInchesPerSec() { return ((getRightSpeed() /
   * ElectricalConstants.DRIVE_TO_INCHES) * 10); }
   * 
   * public double getAverageVelInchesPerSec() { return
   * (getLeftVelocityInchesPerSec() + getRightVelocityInchesPerSec()) / 2; }
   * 
   * public void resetEncoders() { leftMaster.setSelectedSensorPosition(0, 0, 0);
   * rightMaster.setSelectedSensorPosition(0, 0, 0); }
   */

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
      boolean relative, boolean high) {
    if (high) {
      this.shiftHigh();
      drivePID.changePIDGains(Robot.kP_DRIVE * 0.1, Robot.kI_DRIVE, Robot.kD_DRIVE * 5);
    } else {
      this.shiftLow();
      drivePID.changePIDGains(Robot.kP_DRIVE, Robot.kI_DRIVE, Robot.kD_DRIVE);
    }
    if (relative) {
      turnPID.changePIDGains(Robot.kP_DRIVETURN, Robot.kI_DRIVETURN, Robot.kD_DRIVETURN);
    } else {
      turnPID.changePIDGains(Robot.kP_VISION, Robot.kI_VISION, Robot.kD_VISION);
    }

    double currentVal;

    if (relative) {
      currentVal = getYaw();
    } else {
      currentVal = getAngle();
    }

    double driveOut = drivePID.calcPIDDrive(distSetpoint, getAveragePos(), epsilon);
    // limit driving PID output to top speed
    if (driveOut > 0) // driving forward
      driveOut = Math.min(driveOut, topSpeed);
    else if (driveOut < 0) // driving backward
      driveOut = Math.max(driveOut, -topSpeed);

    double angleOut = turnPID.calcPIDDrive(angleSetpoint, currentVal, 1);

    if (relative) {
      if ((angleSetpoint < -90 || angleSetpoint > 90) && currentVal < 0) {
        angleOut = -angleOut;
      }
    }

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
  public void regulatedTurnPID(double angleSetpoint, double epsilon, double topSpeed, boolean relative) {
    if (relative) {
      turnPID.changePIDGains(Robot.kP_TURN, Robot.kI_TURN, Robot.kD_TURN);
    } else {
      turnPID.changePIDGains(Robot.kP_VISION, Robot.kI_VISION, Robot.kD_VISION);
    }
    this.shiftLow(); 

    double currentVal;
    if (relative) {
      currentVal = getYaw();
    } else {
      currentVal = getAngle();
    }

    double angleOut = turnPID.calcPIDDrive(angleSetpoint, currentVal, epsilon);
    if (relative) {
      if ((angleSetpoint < -90 || angleSetpoint > 90) && currentVal < 0) {
        angleOut = -angleOut;
      }
    }
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