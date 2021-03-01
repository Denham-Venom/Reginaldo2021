// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private TalonFX motorTopLeft = new TalonFX(Constants.MOTOR_TOP_LEFT_ID);
  private TalonFX motorBottomLeft = new TalonFX(Constants.MOTOR_BOTTOM_LEFT_ID);
  private TalonFX motorTopRight = new TalonFX(Constants.MOTOR_TOP_RIGHT_ID);
  private TalonFX motorBottomRight = new TalonFX(Constants.MOTOR_BOTTON_RIGHT_ID);

  // private double maxMPVelocity;
  // private double maxMPAcceleration;
  private double feet;
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

  // TrapezoidProfile.Constraints MPconstraints = new TrapezoidProfile.Constraints(maxMPVelocity, maxMPAcceleration);
  // TrapezoidProfile.State MPgoal = new TrapezoidProfile.State();
  // TrapezoidProfile.State MPsetpoint = new TrapezoidProfile.State();
  // TrapezoidProfile profile = new TrapezoidProfile(MPconstraints, MPsetpoint, MPgoal);
  // ProfiledPIDController pidController = new ProfiledPIDController(movementP, movementI, movementD, MPconstraints);

  private static ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
  static final NetworkTableEntry movementF = tuning.add("Movement F", 0).getEntry();
  static final NetworkTableEntry movementP = tuning.add("Movement P", 0).getEntry();
  static final NetworkTableEntry movementI = tuning.add("Movement I", 0).getEntry();
  static final NetworkTableEntry movementD = tuning.add("Movement D", 0).getEntry();
  static final NetworkTableEntry movementFeet = tuning.add("Amount to move in Feet", 0).getEntry();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motorConfig();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("DT Inverted?", inverted);
    SmartDashboard.putBoolean("In High Gear?", isHighGear);

    feet = movementFeet.getDouble(0);

    if(movementF.getDouble(0) != f) {
      f = movementF.getDouble(0);
      motorTopLeft.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
      motorTopRight.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
    }
    if(movementP.getDouble(0) != p) {
      p = movementP.getDouble(0);
      motorTopLeft.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
      motorTopRight.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
    }
    if(movementI.getDouble(0) != i) {
      i = movementI.getDouble(0);
      motorTopLeft.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
      motorTopRight.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
    }
    if(movementD.getDouble(0) != d) {
      d = movementD.getDouble(0);
      motorTopLeft.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);
      motorTopRight.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);
    }
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

    // ----------------❤---------------------
    // Acceleration Ramping :)
    motorTopLeft.configOpenloopRamp(0.25);
    motorTopLeft.configClosedloopRamp(0);
    motorTopRight.configOpenloopRamp(0.25);
    motorTopRight.configClosedloopRamp(0);
    //-----------------❤---------------------

    // Motors follow each other
    motorBottomLeft.follow(motorTopLeft);
    motorBottomRight.follow(motorTopRight);
    // --------------------------------------
    motorTopLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMS);
    motorTopRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMS);

    f = movementF.getDouble(0);
    p = movementP.getDouble(0);
    i = movementI.getDouble(0);
    d = movementD.getDouble(0);
    feet = SmartDashboard.getEntry("Auto Move in feet").getDouble(10.0);
    motorTopLeft.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
    motorTopLeft.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
    motorTopLeft.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
    motorTopLeft.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);
    motorTopRight.config_kF(Constants.kPIDLoopIdx, f, Constants.kTimeoutMS);
    motorTopRight.config_kP(Constants.kPIDLoopIdx, p, Constants.kTimeoutMS);
    motorTopRight.config_kI(Constants.kPIDLoopIdx, i, Constants.kTimeoutMS);
    motorTopRight.config_kD(Constants.kPIDLoopIdx, d, Constants.kTimeoutMS);

    motorTopLeft.configAllowableClosedloopError(Constants.kPIDLoopIdx, Constants.allowableCloseLoopError, Constants.kTimeoutMS);
    motorTopRight.configAllowableClosedloopError(Constants.kPIDLoopIdx, Constants.allowableCloseLoopError, Constants.kTimeoutMS);
    // motorTopLeft.configCurrent
  }

  /**
   * Sets left motors of drivetrain with percent output
   * 
   * @param output percent [-1.0, 1.0]
   */
  public void setLeftMotors(double output) {
    motorTopLeft.set(TalonFXControlMode.PercentOutput, output);
  }

  public void setRightMotors(double output) {
    motorTopRight.set(TalonFXControlMode.PercentOutput, output);
  }

  public void stopMotors() {
    setLeftMotors(0);
    setRightMotors(0);
  }

  public double amountToMove() {
    return feet * Constants.FEET_TO_ROT_UNITS;
  }

  // public void goToStart() {
  //   pidController.setGoal(0);
  // }

  // public void goToEnd() {
  //   pidController.setGoal(5);
  // }

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

  public void setWithPostion() {
    motorTopLeft.set(TalonFXControlMode.Position, motorTopLeft.getSelectedSensorPosition() + amountToMove());
    motorTopRight.set(TalonFXControlMode.Position, motorTopRight.getSelectedSensorPosition() + amountToMove());
  }

  public void setWithPostion(double feet) {
    this.feet = feet;
    motorTopLeft.set(TalonFXControlMode.Position, amountToMove());
    motorTopRight.set(TalonFXControlMode.Position, amountToMove());
  }
}      
