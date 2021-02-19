// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  private final TalonFX motorTopLeft = new TalonFX(Constants.MOTOR_TOP_LEFT_ID);
  private TalonFX motorBottomLeft = new TalonFX(Constants.MOTOR_BOTTOM_LEFT_ID);
  private TalonFX motorTopRight = new TalonFX(Constants.MOTOR_TOP_RIGHT_ID);
  private TalonFX motorBottomRight = new TalonFX(Constants.MOTOR_BOTTON_RIGHT_ID);

  private static double feet;
  private double f;
  private double p;
  private double i;
  private double d;
  private boolean inverted = false;
  private boolean isHighGear;
  private double highGear = Constants.DT_HG;// 1;
  private double highGearTurn = Constants.DT_HGT;// .9;
  private double lowGear = Constants.DT_LG;// 0.8;
  private double lowGearTurn = Constants.DT_LGT;// 0.65;

  // private static Shuffleboard tuning = Shuffleboard.getTab("Tuning");
  // static final NetworkTableEntry movementF = tuning.add("Movement F",
  // 0).getEntry();
  // static final NetworkTableEntry movementP = tuning.add("Movement P",
  // 0).getEntry();
  // static final NetworkTableEntry movementI = tuning.add("Movement I",
  // 0).getEntry();
  // static final NetworkTableEntry movementD = tuning.add("Movement D",
  // 0).getEntry();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motorConfig();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("DT Inverted?", inverted);
    SmartDashboard.putBoolean("In High Gear?", isHighGear);
  }

  private void motorConfig() {
    // Sets all motors to factory default.
    motorTopLeft.configFactoryDefault();
    motorTopRight.configFactoryDefault();
    motorBottomLeft.configFactoryDefault();
    motorBottomRight.configFactoryDefault();
    // --------------------------------------------

    // Netutral Mode brake for staying still
    motorTopLeft.setNeutralMode(NeutralMode.Brake);
    motorTopRight.setNeutralMode(NeutralMode.Brake);
    motorBottomLeft.setNeutralMode(NeutralMode.Brake);
    motorBottomRight.setNeutralMode(NeutralMode.Brake);
    // --------------------------------------------------

    // Inverted so the robot doesn't spin
    motorTopLeft.setInverted(false);
    motorTopRight.setInverted(true);
    motorBottomLeft.setInverted(false);
    motorBottomRight.setInverted(true);
    // -------------------------------------

    // Motors follow each other
    motorBottomLeft.follow(motorTopLeft);
    motorBottomRight.follow(motorTopRight);
    // --------------------------------------
    motorTopLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMS);
    motorTopRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMS);

    // f = movementF.getDouble(0);
    // p = movementP.getDouble(0);
    // i = movementI.getDouble(0);
    // d = movementD.getDouble(0);
    feet = SmartDashboard.getEntry("Auto Move in feet").getDouble(10.0);
    motorTopLeft.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
    motorTopLeft.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
    motorTopLeft.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
    motorTopLeft.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);
    motorTopRight.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
    motorTopRight.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
    motorTopRight.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
    motorTopRight.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);
  }

  /**
   * Sets left motors of drivetrain with percent output
   * 
   * @param output percent [-1.0, 1.0]
   */
  public void setLeftMotors(double output) {
    motorTopLeft.set(ControlMode.PercentOutput, output);
  }

  public void setRightMotors(double output) {
    motorTopRight.set(ControlMode.PercentOutput, output);
  }

  public void stopMotors() {
    setLeftMotors(0);
    setRightMotors(0);
  }

  //public static double amountToMove() {
  //  return feet * Constants.FEET_TO_ROT_UNITS;
  //}

  

  public void invertDrive() {
    inverted = !inverted;
  }

  public boolean getInverted() {
    return inverted;
  }

  public void hLGearSwitch() {
    // switches the robot into low and high gear (high = high speed, low = low speed)
    if(isHighGear) {
      isHighGear = false;
    }else{
      isHighGear = true;
    }
  }

  public boolean getGearSwitch() {
    // returns whether the robot is in high or low gear
    return isHighGear;
  }

  public double[] highLowGearValue() {
    // if the robot is in high gear, it applies the variable
    // highGear and highGearTurn to the robot
    // if the robot is in low gear, it applies the variable 
    // lowGear and lowGearTurn to the robot
    if(isHighGear) {
      double[] arr = {highGear,highGearTurn};
      return arr;
    } else {
      double[] arr = {lowGear,lowGearTurn};
      return arr;
    }
  }
}      
