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
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.ElectricalConstants;
import frc.robot.commands.carriage.CarriageCommand;

public class Carriage extends Subsystem {

  public static Carriage mInstance; 

  //Victor motors
  VictorSPX feederMotor;
  VictorSPX shooterMotorLeft;
  VictorSPX shooterMotorRight;

  //Solonoids
  DoubleSolenoid clawSolenoid;
  Solenoid ejectorSolenoid;
  DoubleSolenoid sliderSolenoid;

  //Digital inputs
  DigitalInput hatchSensor;
  DigitalInput cargoOptical; 

  //RangeFinder
  Ultrasonic hatchDetectorRight; 
  Ultrasonic hatchDetectorLeft; 

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
    feederMotor.setInverted(true);

    shooterMotorLeft = new VictorSPX(ElectricalConstants.SHOOTER_MOTOR_LEFT);
    shooterMotorLeft.setInverted(true);
    shooterMotorRight = new VictorSPX(ElectricalConstants.SHOOTER_MOTOR_RIGHT);

    //Intializes the solenoids
    clawSolenoid= new DoubleSolenoid(ElectricalConstants.CLAW_SOLENOID_A, ElectricalConstants.CLAW_SOLENOID_B);
    ejectorSolenoid = new Solenoid(ElectricalConstants.EJECTOR_SOLENOID_B);
    sliderSolenoid= new DoubleSolenoid(ElectricalConstants.SLIDER_SOLENOID_A, ElectricalConstants.SLIDER_SOLENOID_B);

    cargoOptical = new DigitalInput(ElectricalConstants.CARGO_DETECTOR_CARRIAGE);

    //hatchDetectorRight = new Ultrasonic(ElectricalConstants.HATCH_PANEL_DETECTOR_TRIGGER_RIGHT, ElectricalConstants.HATCH_PANEL_DETECTOR_ECHO_RIGHT); 
    hatchDetectorLeft = new Ultrasonic(ElectricalConstants.HATCH_PANEL_DETECTOR_TRIGGER_LEFT, ElectricalConstants.HATCH_PANEL_DETECTOR_ECHO_LEFT);
    //hatchDetectorRight.setAutomaticMode(true);
    hatchDetectorLeft.setAutomaticMode(true);

    // retractCarriage();
    // ejectHatch();
    // holdAndSecure();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CarriageCommand());
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
    shooterMotorRight.set(ControlMode.PercentOutput,-val);
  }

  //Shoots cargo backwards at full speed(-1)
  public void shootBackwards(double val)
  {
    shooterMotorLeft.set(ControlMode.PercentOutput,-val);
    shooterMotorRight.set(ControlMode.PercentOutput,val);
  }

  //Retracts solenoids
  public void holdAndSecure()
  {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  //Extends solenoids
  public void prisonBreak()
  {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean getClaw() {
    if (clawSolenoid.get() == DoubleSolenoid.Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public void extendCarriage()
  {
    sliderSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractCarriage()
  {
    sliderSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void ejectHatch()
  {
    ejectorSolenoid.set(false);
   //ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void retractEjector()
  {
    ejectorSolenoid.set(true);
    //ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
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
    //no exclamation on practice
    return cargoOptical.get();
  }

  // public double getUltrasonicRight(){
  //   return hatchDetectorRight.getRangeInches(); 
  // }
  public double getRangeDist(){
    return hatchDetectorLeft.getRangeInches(); 
  }

}