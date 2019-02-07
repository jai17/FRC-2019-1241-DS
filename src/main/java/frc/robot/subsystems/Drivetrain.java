package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.ElectricalConstants;
import frc.robot.NumberConstants;
import frc.robot.RobotState;
import frc.robot.PID.PIDController;
import frc.robot.commands.drive.TankDrive;
import frc.robot.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

  //Drive instance
  private static Drivetrain mInstance; 
  //Robot State 
  private RobotState robotState; 
  //Talon Control Mode constant
  private ControlMode talonControlMode = ControlMode.PercentOutput; 

  /* Drive speed controlers */
  public TalonSRX leftMaster;
  private VictorSPX leftSlave1;
  private VictorSPX leftSlave2;

  private TalonSRX rightMaster;
  private VictorSPX rightSlave1;
  private VictorSPX rightSlave2;

  /* Encoders on the drive */
  private boolean leftEncoderConnected = false;
  private boolean rightEncoderConnected = false;  

  /** Gyro on the drive */
  AHRS gyro;

  /*PID*/
  private PIDController drivePID;
  private PIDController turnPID;
  PIDController gyroPID; 
  private PIDController lockPID;

  /* Solenoid Shifter */
  private DoubleSolenoid shifterSolenoid;
  private boolean isLow = false;

  //Create a single instance of the drivetrain
  public static Drivetrain getInstance() {
		if (mInstance == null) {
			mInstance = new Drivetrain();
			return mInstance;
		} else
			return mInstance;
  }
  

public Drivetrain() {
  //Initialize Talons
  //left master
  leftMaster = new TalonSRX(ElectricalConstants.LEFT_DRIVE_FRONT);
  leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
  leftMaster.setInverted(false);
  leftMaster.setSensorPhase(false);
  leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

  //left slaves
  leftSlave1 = new VictorSPX(ElectricalConstants.LEFT_DRIVE_BACK);
  leftSlave1.set(ControlMode.Follower, ElectricalConstants.LEFT_DRIVE_FRONT);
  leftSlave1.follow(leftMaster); 
  leftSlave2 = new VictorSPX(ElectricalConstants.LEFT_DRIVE_MIDDLE);
  leftSlave2.set(ControlMode.Follower, ElectricalConstants.LEFT_DRIVE_FRONT);
  leftSlave2.follow(leftMaster);

  //right masters
  rightMaster = new TalonSRX(ElectricalConstants.RIGHT_DRIVE_FRONT);
  rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
  rightMaster.setInverted(false);
  rightMaster.setSensorPhase(true);
  rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

  //right slaves
  rightSlave2 = new VictorSPX(ElectricalConstants.RIGHT_DRIVE_MIDDLE);
  rightSlave2.set(ControlMode.Follower, ElectricalConstants.RIGHT_DRIVE_FRONT);
  rightSlave2.follow(rightMaster);
  rightSlave1 = new VictorSPX(ElectricalConstants.RIGHT_DRIVE_BACK);
  rightSlave1.set(ControlMode.Follower, ElectricalConstants.RIGHT_DRIVE_FRONT);
  rightSlave1.follow(rightMaster);

   //For Motion Magic set up 
   rightMaster.selectProfileSlot(0, 0);
   rightMaster.configMotionAcceleration(2500, 0); 
   rightMaster.configMotionCruiseVelocity(2500, 0);
   rightMaster.config_kF(0, NumberConstants.fTalonDrive, 10);
   rightMaster.config_kP(0, NumberConstants.pTalonDrive, 0);
   rightMaster.config_kI(0, NumberConstants.iTalonDrive, 0);
   rightMaster.config_kD(0, NumberConstants.dTalonDrive, 0);

   //For Motion Magic set up 
   leftMaster.selectProfileSlot(0, 0);
   leftMaster.configMotionAcceleration(2500, 0); 
   leftMaster.configMotionCruiseVelocity(2500, 0);
   leftMaster.config_kF(0, NumberConstants.fTalonDrive, 10);
   leftMaster.config_kP(0, NumberConstants.pTalonDrive, 0);
   leftMaster.config_kI(0, NumberConstants.iTalonDrive, 0);
   leftMaster.config_kD(0, NumberConstants.dTalonDrive, 0);

  //Initialize PID controllers
  drivePID = new PIDController(NumberConstants.pDrive, NumberConstants.iDrive, NumberConstants.dDrive);
  turnPID = new PIDController(NumberConstants.pTurn, NumberConstants.iTurn, NumberConstants.dTurn);
  lockPID = new PIDController(NumberConstants.pLock, NumberConstants.iLock, NumberConstants.dLock);
  gyroPID = new PIDController(NumberConstants.pGyro, NumberConstants.iGyro, NumberConstants.dGyro); 

  //Initialize shifter solenoid
  shifterSolenoid = new DoubleSolenoid(ElectricalConstants.SHIFTER_SOLENOID_HIGH, ElectricalConstants.SHIFTER_SOLENOID_LOW);

  //Initialize navX gyro
  try {
    gyro = new AHRS(SPI.Port.kMXP);
  } catch (RuntimeException ex) {
    DriverStation.reportError("ERROR navX: " + ex.getMessage(), true);
  }
  
  //reset encoders, gyro, shift high
  reset();
  shiftHigh();
}

  @Override
  public void initDefaultCommand() {
		setDefaultCommand(new TankDrive());
  }
  
  public void runLeftDrive(double input) {
    leftMaster.set(ControlMode.PercentOutput, input);
  }

  public void runRightDrive(double input) {
    rightMaster.set(ControlMode.PercentOutput, input);
  }

  //encoder methods
  public double getLeftPos() {
	  return leftMaster.getSelectedSensorPosition(0) / ElectricalConstants.DRIVE_TO_INCHES;
  }

  public double getRightPos() {
		return rightMaster.getSelectedSensorPosition(0) / ElectricalConstants.DRIVE_TO_INCHES;
  }

  public double getAvgPos() {
    return (getLeftPos() + getRightPos())/2;
  }

  public double getLeftSpeed() {
		return leftMaster.getSelectedSensorVelocity(0);
	}

	public double getRightSpeed() {
		return rightMaster.getSelectedSensorVelocity(0);
	}

	public double getLeftVelocityInchesPerSec() {
		return ((getLeftSpeed() / ElectricalConstants.DRIVE_TO_INCHES) * 10);
	}

	public double getRightVelocityInchesPerSec() {
		return ((getRightSpeed() / ElectricalConstants.DRIVE_TO_INCHES) * 10);
	}

  public void resetEncoders() {
		leftMaster.setSelectedSensorPosition(0, 0, 0);
		rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  //gyro angles
  public double getAngle() {
		return gyro.getAngle();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public void resetGyro() {
    gyro.zeroYaw();
  }

  //solenoid methods
  public DoubleSolenoid.Value getIslow() {
    return shifterSolenoid.get();
  }

  public void shiftLow() {
    shifterSolenoid.set(DoubleSolenoid.Value.kForward);
    isLow = true;
  }

  public void shiftHigh() {
    shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
    isLow = false;
  }

  public boolean getIsLow() {
    return isLow;
  }

  /*************** PID methods *******************/
  
	public void driveSetpoint(double setPoint, double speed, double setAngle, double tolerance) {
		double output = drivePID.calcPID(setPoint, getAvgPos(), tolerance);
		double angle = gyroPID.calcPID(setAngle, getYaw(), tolerance);
		double min = 0.20;
		
		if(output < min && output > 0){
			runLeftDrive((min + angle) * speed);
			runRightDrive((-min + angle) * speed * 0.95);
		}else if(output > -min && output < 0){
			runLeftDrive((-min + angle) * speed);
			runRightDrive((min + angle) * speed * 0.95);
		}
		else{
			runLeftDrive((output + angle) * speed);
			runRightDrive((-output + angle) * speed * 0.95);
		}
	}

	public void turnDrive(double setAngle, double speed, double tolerance) {
		double angle = gyroPID.calcPID(setAngle, getYaw(), tolerance);
		double min = 0.15;

		if (Math.abs(setAngle - getYaw()) < tolerance) {
			runLeftDrive(0);
			runRightDrive(0);
		} else if (angle > -min && angle < 0) {
			runLeftDrive(-min);
			runRightDrive(-min);
		} else if (angle < min && angle > 0) {
			runLeftDrive(min);
			runRightDrive(min);
		} else {
			runLeftDrive(-angle * speed);
			runRightDrive(-angle * speed);
		}
	}

	public void driveAngle(double setAngle, double speed) {
		double angle = gyroPID.calcPID(setAngle, getYaw(), 1);

		runLeftDrive(speed + angle);
		runRightDrive(-speed + angle);
	}

	public boolean drivePIDDone() {
		return drivePID.isDone();
  }
  public void resetDrivePID(){
    drivePID.resetPID();
  }
	public boolean gyroPIDDone() {
		return gyroPID.isDone();
  }
  public void resetGyroPID(){
    gyroPID.resetPID();
  }

	public void changeDriveGains(double p, double i, double d) {
		drivePID.changePIDGains(p, i, d);
	}

	public void changeGyroGains(double p, double i, double d) {
		gyroPID.changePIDGains(p, i, d);
	}

  
  /************ Motion Magic Methods *******/
  public void runDriveMotionMagic(double setpoint) {
    rightMaster.set(ControlMode.MotionMagic, setpoint);
    leftMaster.set(ControlMode.MotionMagic, setpoint);
  }

  public double getMotionMagicError() {
    return (rightMaster.getClosedLoopError(0) + leftMaster.getClosedLoopError())/2;
  }

  public void magicMotionSetpoint(double setpoint, int cruiseVelocity, double secsToMaxSpeed) {
    rightMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
    rightMaster.configMotionAcceleration((int) (NumberConstants.DRIVE_MAX_SPEED / secsToMaxSpeed), 0);

    leftMaster.configMotionCruiseVelocity(cruiseVelocity, 0);
    leftMaster.configMotionAcceleration((int) (NumberConstants.DRIVE_MAX_SPEED / secsToMaxSpeed), 0);

    runDriveMotionMagic(setpoint * -ElectricalConstants.DRIVE_TO_INCHES);
  }

  /*********** Ramp Rates ****************/
  public void setLeftRampRate(double rampRate) {
		leftMaster.set(ControlMode.Velocity, rampRate);
	}

	public void setRightRampRate(double rampRate) {
		rightMaster.set(ControlMode.Velocity, rampRate);
	}

  public void reset() {
		resetEncoders();
		resetGyro();
  }
}