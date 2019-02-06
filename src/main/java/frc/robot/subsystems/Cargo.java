package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.NumberConstants;
import frc.robot.PID.PIDController;
import frc.robot.ElectricalConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Cargo extends Subsystem {

  public static Cargo mInstance; 

  //Speed Controllers
  TalonSRX cargoPivot; //Pivot Talon
  VictorSPX pivotSlave; 
  VictorSPX cargoRoller; //Roller Victor

  //Sensor Feedback 
  DigitalInput limitSwitch; //Pivot limit switch 
  private DigitalInput optical; //Optical Sensor

  //Store state of ball in Cargo intake
  private boolean contains = false;

  //PID Controller
  private PIDController pivotPID; 

   //Create a single instance of the Cargo
   public static Cargo getInstance() {
		if (mInstance == null) {
			mInstance = new Cargo();
			return mInstance;
		} else
			return mInstance;
  }

  public Cargo() {
    cargoPivot = new TalonSRX(ElectricalConstants.CARGO_PIVOT_LEFT);
    cargoPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    cargoPivot.setInverted(false);
    cargoPivot.setSensorPhase(true);
    cargoPivot.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

    //For Motion Magic set up 
    cargoPivot.selectProfileSlot(0, 0);
    cargoPivot.configMotionAcceleration(2500, 0); 
    cargoPivot.configMotionCruiseVelocity(2500, 0);
    cargoPivot.config_kF(0, NumberConstants.fTalonCargo, 10);
    cargoPivot.config_kP(0, NumberConstants.pTalonCargo, 0);
    cargoPivot.config_kI(0, NumberConstants.iTalonCargo, 0);
    cargoPivot.config_kD(0, NumberConstants.dTalonCargo, 0);

    pivotSlave = new VictorSPX(ElectricalConstants.CARGO_PIVOT_RIGHT); 
    pivotSlave.set(ControlMode.Follower, ElectricalConstants.CARGO_PIVOT_LEFT);
    pivotSlave.follow(cargoPivot); 

    //FOR SOFT LIMITS
    cargoPivot.configForwardSoftLimitEnable(true, 0);
    cargoPivot.configForwardSoftLimitThreshold(NumberConstants.CARGO_FORWARD_SOFT_LIMIT, 0); 
    cargoPivot.configReverseSoftLimitEnable(true, 0);
    cargoPivot.configReverseSoftLimitThreshold(NumberConstants.CARGO_BACK_SOFT_LIMIT, 0);

    cargoRoller = new VictorSPX(ElectricalConstants.CARGO_ROLLER_MOTOR);

    optical = new DigitalInput(ElectricalConstants.CARGO_OPTICAL);
    limitSwitch = new DigitalInput(ElectricalConstants.CARGO_LIMIT_SWITCH);

    pivotPID = new PIDController(NumberConstants.pCargo, NumberConstants.iCargo, NumberConstants.dCargo);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // intake, outtake, stop, runUp, runDown
  public void intake(double val) {
    cargoRoller.set(ControlMode.PercentOutput, val);
  }

  // outtake )
  public void outtake(double val) {
    cargoRoller.set(ControlMode.PercentOutput, -val);
  }

  // stop the intake
  public void stop() {
    cargoRoller.set(ControlMode.PercentOutput, 0);
  }

  // run (something) up at positive output
  public void runUp(double output) {
    cargoPivot.set(ControlMode.PercentOutput, output);
  }

  // run (something) down at negative output
  public void runDown(double output) {
    cargoPivot.set(ControlMode.PercentOutput, -output);
  }

  // getAngles, resetAngles, isCargoPresent
  public double getRawCargoPivot() {
    return cargoPivot.getSelectedSensorPosition(0); 
  }

  //Returns the Cargo pivot angle in degrees 
  public double getCargoAngle(){
    return getRawCargoPivot()/ElectricalConstants.CARGO_TO_DEGREES; 
  }

  //Resets the Cargo pivot angle 
  public void resetAngle() {
    cargoPivot.setSelectedSensorPosition(0);
  }

  //Sets the state of the ball in the Cargo intake
  public void setContains(boolean state) { 
    this.contains = state;
  }

  //Returns if the cargo is present
  public boolean isCargoPresent() { 
    return contains;
  }

  //Gets the state of the Cargo from the sensor 
  public boolean getOptic() { 
    return optical.get();
  }

  //********************PID Functions *********************/
  public void changeCargoPIDGain(double p, double i , double d){
    pivotPID.changePIDGains(p, i, d);
  }

  //Reset the Cargo pivot PID
  public void resetPID(){
    pivotPID.resetPID();
  }

  /*Method to run the PID for the Cargo pivot
  *@param angle
  *@param speed
  *@param tolerance
  **/
  public void pivotSetpoint (double angle, double speed, double tolerance){
    double output = pivotPID.calcPID(angle, getCargoAngle(), tolerance); 
    if (output>=0){
      runUp(output);
    } else {
      runDown(output);
    }
  }


  public void runCargoMotionMagic(double setpoint) {
    cargoPivot.set(ControlMode.MotionMagic, setpoint);
  }

  public double getMotionMagicError() {
    return cargoPivot.getClosedLoopError(0);
  }

  public void magicMotionSetpoint(double setpoint, int cruiseVelocity, double secsToMaxSpeed) {
    cargoPivot.configMotionCruiseVelocity(cruiseVelocity, 0);
    cargoPivot.configMotionAcceleration((int) (NumberConstants.CARGO_MAX_SPEED / secsToMaxSpeed), 0);
    runCargoMotionMagic(setpoint * -ElectricalConstants.CARGO_TO_DEGREES);
  }

  // Pivot limit switch
  public boolean limitSwitch() {
    return limitSwitch.get();
  }
}