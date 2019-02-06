/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ElectricalConstants;
import frc.robot.NumberConstants;
import frc.robot.PID.PIDController;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends Subsystem {

  public static Elevator mInstance; 

  //Talons 
  private TalonSRX tachTalon; 
  private TalonSRX magEncoderTalon;
  private boolean encoderConnected;

  PIDController elevatorPID; 

   //Create a single instance of the Elevator
   public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator();
			return mInstance;
		} else
			return mInstance;
  }

  public Elevator(){

    magEncoderTalon = new TalonSRX(ElectricalConstants.RIGHT_ELEVATOR_MOTOR);

    //Set up Mag Encoders
    magEncoderTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		magEncoderTalon.setInverted(false);
		magEncoderTalon.setSensorPhase(true);
    magEncoderTalon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
  
    // Method in order to set a default Motion Magic Velocity and Acceleration 
    magEncoderTalon.configMotionAcceleration(2500, 0);
    magEncoderTalon.configMotionCruiseVelocity(2500, 0);

    magEncoderTalon.config_kP(0, NumberConstants.pTalonElevator, 0);
    magEncoderTalon.config_kI(0, NumberConstants.iTalonElevator, 0);
    magEncoderTalon.config_kD(0, NumberConstants.dTalonElevator, 0);
    magEncoderTalon.config_kF(0, NumberConstants.fTalonElevator, 0);
    
    tachTalon = new TalonSRX(ElectricalConstants.LEFT_ELEVATOR_MOTOR);
    tachTalon.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, 0); 
    tachTalon.set(ControlMode.Follower, ElectricalConstants.LEFT_ELEVATOR_MOTOR);
    tachTalon.follow(magEncoderTalon);

    elevatorPID = new PIDController(NumberConstants.pElevator, NumberConstants.iElevator, NumberConstants.dElevator);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean getAtBottom(){
    return tachTalon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public void runUp(double val){
    magEncoderTalon.set(ControlMode.PercentOutput,val);
  }

  public void runDown(double val){
    magEncoderTalon.set(ControlMode.PercentOutput,-val);
  }

  public void runElevator(double val){
    magEncoderTalon.set(ControlMode.PercentOutput, val);
  }

public void runElevatorMotionMagic(double setpoint){
  magEncoderTalon.set(ControlMode.MotionMagic, setpoint);
}

public void setSetpoint(double power){
    magEncoderTalon.set(ControlMode.PercentOutput,power);
}

public void setMotionMagicSetpoint(double setpoint, int cruiseVelocity, double secsToMaxSpeed) {
		
		magEncoderTalon.configMotionCruiseVelocity(cruiseVelocity, 0);
		magEncoderTalon.configMotionAcceleration((int)(NumberConstants.ELEVATOR_MAX_SPEED/secsToMaxSpeed), 0);

		runElevatorMotionMagic(setpoint*NumberConstants.ELEVATOR_MAX_SPEED);
}

// ************************** PID Functions ******************************
public void changeElevatorGains(double p, double i, double d) {
  elevatorPID.changePIDGains(p, i, d);
}

public void resetPID() {
  elevatorPID.resetPID();
}

public void elevatorSetpoint(double setPoint, double speed, double tolerance) {
  double output = elevatorPID.calcPID(setPoint, getElevatorEncoder(), tolerance);

  runElevator(output * speed);
}

// ************************Mag Encoder Functions************************
public boolean isEncoderConnected() {
  return encoderConnected;
}

public double getElevatorEncoder() {
  return magEncoderTalon.getSelectedSensorPosition(0) / ElectricalConstants.ELEVATOR_TO_INCHES;
}

public double getElevatorRotations() {
  return magEncoderTalon.getSelectedSensorPosition(0);
}

public double getElevatorSpeed() {
  return magEncoderTalon.getSelectedSensorVelocity(0);
}

public void resetEncoders() {
  magEncoderTalon.setSelectedSensorPosition(0, 0, 0);
}

public double elevatorCurrentDraw(){
  return magEncoderTalon.getOutputCurrent();
}
}