/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.ElectricalConstants;
import frc.robot.NumberConstants;
import frc.robot.PID.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Hatch extends Subsystem {

  public static Hatch mInstance; 

  TalonSRX hatchPivot;
  VictorSPX hatchRoller;

  DigitalInput hatchLimit;
  DigitalInput hatchDetector;

  PIDController pivotPID; 

   //Create a single instance of the Hatch
   public static Hatch getInstance() {
		if (mInstance == null) {
			mInstance = new Hatch();
			return mInstance;
		} else
			return mInstance;
  }

  public Hatch() {
    hatchPivot = new TalonSRX(ElectricalConstants.HATCH_PIVOT_MOTOR);
    hatchPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    hatchPivot.setInverted(false);
    hatchPivot.setSensorPhase(false);

    hatchRoller = new VictorSPX(ElectricalConstants.HATCH_ROLLER);

    // hatchLimit = new DigitalInput(ElectricalConstants.HATCH_LIMIT_SWITCH);
    // hatchDetector = new DigitalInput(ElectricalConstants.HATCH_PANEL_DETECTOR_INTAKE);

    // For Motion Magic set up
    hatchPivot.selectProfileSlot(0, 0);
    hatchPivot.configMotionAcceleration(2500, 0);
    hatchPivot.configMotionCruiseVelocity(2500, 0);
    hatchPivot.config_kF(0, NumberConstants.fTalonHatch, 10);
    hatchPivot.config_kP(0, NumberConstants.pTalonHatch, 0);
    hatchPivot.config_kI(0, NumberConstants.iTalonHatch, 0);
    hatchPivot.config_kD(0, NumberConstants.dTalonHatch, 0);

    // FOR SOFT LIMITS
    hatchPivot.configForwardSoftLimitEnable(true, 0);
    hatchPivot.configForwardSoftLimitThreshold(NumberConstants.HATCH_FORWARD_SOFT_LIMIT, 0);
    hatchPivot.configReverseSoftLimitEnable(true, 0);
    hatchPivot.configReverseSoftLimitThreshold(NumberConstants.HATCH_BACK_SOFT_LIMIT, 0);

    pivotPID = new PIDController(NumberConstants.pHatch, NumberConstants.iHatch, NumberConstants.dHatch);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Intake
  public void intake(double val) {
    hatchRoller.set(ControlMode.PercentOutput, val);
  }

  // Outtake
  public void outtake(double val) {
    hatchRoller.set(ControlMode.PercentOutput, -val);
  }

  // Hatch is off (0)
  public void stop(double output) {
    hatchRoller.set(ControlMode.PercentOutput, 0);
  }

  public void pivotHatch(double output){
    hatchPivot.set(ControlMode.PercentOutput, output);
  }

  /************ Mag Encoder Functions **************/

  public double getRawHatchPivot() {
    return hatchPivot.getSelectedSensorPosition(0);
  }

  public double getAngle() {
    return hatchPivot.getSelectedSensorPosition(0) / ElectricalConstants.HATCH_TO_DEGREES;
  }

  public void resetAngle() {
    hatchPivot.setSelectedSensorPosition(0, 0, 0);
  }

  //********************PID Functions *********************/

  public void changeCargoPIDGain(double p, double i , double d){
    pivotPID.changePIDGains(p, i, d);
  }

  //Reset the Cargo pivot PID
  public void resetPID(){
    pivotPID.resetPID();
  }

  /*Method to run the PID for the Hatch pivot
  *@param angle
  *@param speed
  *@param tolerance
  **/
  public void pivotSetpoint (double angle, double speed, double tolerance){
    double output = pivotPID.calcPID(angle, getAngle(), tolerance); 
    pivotHatch(output);
  }


  public void runHatchMotionMagic(double setpoint) {
    hatchPivot.set(ControlMode.MotionMagic, setpoint);
  }

  public double getMotionMagicError() {
    return hatchPivot.getClosedLoopError(0);
  }

  public void magicMotionSetpoint(double setpoint, int cruiseVelocity, double secsToMaxSpeed) {
    hatchPivot.configMotionCruiseVelocity(cruiseVelocity, 0);
    hatchPivot.configMotionAcceleration((int) (NumberConstants.HATCH_MAX_SPEED / secsToMaxSpeed), 0);
    runHatchMotionMagic(setpoint * -ElectricalConstants.HATCH_TO_DEGREES);
  }

  // public boolean getIsBack() {
  //   return hatchLimit.get();
  // }
}