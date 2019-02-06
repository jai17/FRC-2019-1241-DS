/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ElectricalConstants;

public class Carriage extends Subsystem {

  public static Carriage mInstance; 

  //Victor motors
  VictorSPX feederMotor;
  VictorSPX shooterMotorLeft;
  VictorSPX shooterMotorRight;

  //Solonoids
  Solenoid clawSolenoid;
  Solenoid ejectorSolenoid;
  Solenoid sliderSolenoid;

  //Digital inputs
  DigitalInput hatchSensor;
  DigitalInput cargoOptical; 

   //Store state of ball in Carriage
   private boolean contains = false;

    //Create a single instance of the Carriage
  public static Carriage getInstance() {
		if (mInstance == null) {
			mInstance = new Carriage();
			return mInstance;
		} else
			return mInstance;
  }

  //Constructor
  public Carriage()
  {
    //Intializes the victors
    feederMotor= new VictorSPX(ElectricalConstants.FEEDER_MOTOR);
    shooterMotorLeft= new VictorSPX(ElectricalConstants.SHOOTER_MOTOR_LEFT);
    shooterMotorRight= new VictorSPX(ElectricalConstants.SHOOTER_MOTOR_RIGHT);

    //Intializes the solenoids
    clawSolenoid= new Solenoid(ElectricalConstants.CONE_SOLENOID);
    ejectorSolenoid = new Solenoid( ElectricalConstants.EJECTOR_SOLENOID);
    sliderSolenoid= new Solenoid(ElectricalConstants.SLIDER_SOLENOID);

    cargoOptical = new DigitalInput(ElectricalConstants.CARGO_DETECTOR_CARRIAGE);

  }

  @Override
  public void initDefaultCommand() {
  }

  //Runs the feeder in at full speed(1)
  public void feederIn(double val)
  {
    feederMotor.set(ControlMode.PercentOutput, val);
  }

  //Runs the feeder out at full speed(-1)
  public void feederOut(double val)
  {
    feederMotor.set(ControlMode.PercentOutput,-val);
  }

  //Shoots cargo forwards at full speed(1) 
  public void shootForward(double val)
  {
    shooterMotorLeft.set(ControlMode.PercentOutput,val);
    shooterMotorLeft.set(ControlMode.PercentOutput,-val);
  }

  //Shoots cargo backwards at full speed(-1)
  public void shootBackwards(double val)
  {
    shooterMotorLeft.set(ControlMode.PercentOutput,-val);
    shooterMotorLeft.set(ControlMode.PercentOutput,val);
  }

  //Retracts solenoids
  public void holdAndSecure()
  {
    clawSolenoid.set(true);
  }

  //Extends solenoids
  public void prisonBreak()
  {
    clawSolenoid.set(false);
  }

  public void extendCarriage()
  {
    sliderSolenoid.set(true);
  }

  public void retractCarriage()
  {
    sliderSolenoid.set(false);
  }

  public void ejectHatch()
  {
    ejectorSolenoid.set(true);
  }
  public void retractEjector()
  {
    ejectorSolenoid.set(false);
  }

  //Sets the state of the ball in the Carriage
  public void setContains(boolean state) { 
    this.contains = state;
  }

  //Returns if the cargo is present
  public boolean isCargoPresent() { 
    return contains;
  }

  //Gets the state of the Cargo from the sensor 
  public boolean getOptic() { 
    return cargoOptical.get();
  }

}