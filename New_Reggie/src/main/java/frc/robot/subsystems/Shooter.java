// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

  NetworkTableEntry shootRPM = Shuffleboard.getTab("Tuning").add("RPM", 0).getEntry();
  NetworkTableEntry leftP = Shuffleboard.getTab("Tuning").add("LS VP", 0).getEntry();
  NetworkTableEntry rightP = Shuffleboard.getTab("Tuning").add("RS VP", 0).getEntry();
  double targetRPM = 0;
  double LP;
  double RP;
  boolean tuningEnable = false;

  double speed;
  private final TalonFX shootMotorLeft = new TalonFX(Constants.SHOOT_MOTOR_LEFT_ID);
  private final TalonFX shootMotorRight = new TalonFX(Constants.SHOOT_MOTOR_RIGHT_ID);
  private final CANSparkMax adjustAngleMotorLeft = new CANSparkMax(Constants.ADJUST_ANGLE_MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax adjustAngleMotorRight = new CANSparkMax(Constants.ADJUST_ANGLE_MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final CANEncoder angleEncoder = adjustAngleMotorLeft.getEncoder();
//---------------------------------------------------------------

  /** Creates a new Shooter. */
  public Shooter() {
    configShoot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Angle", angleEncoder.getPosition());
    SmartDashboard.putNumber("L Shoot RPM", getLeftShooterRPM());
    SmartDashboard.putNumber("R Shoot RPM", getRightShooterRPM());

    targetRPM = shootRPM.getDouble(0);
    
    if(Robot.tuningEnable.getBoolean(false)) {
      tuningEnable = true;

      double lastLP = LP;
      LP = leftP.getDouble(0);
      if(lastLP != LP) {
        shootMotorLeft.config_kP(Constants.kSlotIdx, LP, Constants.kTimeMS);
      }

      double lastRP = RP;
      RP = rightP.getDouble(0);
      if(lastRP != RP) {
        shootMotorRight.config_kP(Constants.kSlotIdx, RP, Constants.kTimeMS);
      }
    } else if(!tuningEnable) {
      LP = Constants.SHOOT_LEFT_VEL_P;
      shootMotorLeft.config_kP(Constants.kSlotIdx, LP, Constants.kTimeMS);

      RP = Constants.SHOOT_RIGHT_VEL_P;
      shootMotorRight.config_kP(Constants.kSlotIdx, RP, Constants.kTimeMS);
    }
    
  }

  public void setShooterMotors(double speed) {
    shootMotorLeft.set(ControlMode.PercentOutput, speed);
    shootMotorRight.set(ControlMode.PercentOutput, speed);
  }

  public void setShooterVelocity(double RPM) {
    shootMotorLeft.set(ControlMode.Velocity, RPM * Constants.RPM_TO_TP100MS);
    shootMotorRight.set(ControlMode.Velocity, RPM * Constants.RPM_TO_TP100MS);
  }

  public void setShooterVelocity() {
    shootMotorLeft.set(ControlMode.Velocity, targetRPM * Constants.RPM_TO_TP100MS);
    shootMotorRight.set(ControlMode.Velocity, targetRPM * Constants.RPM_TO_TP100MS);
  }

  public double getLeftShooterRPM() {
    return shootMotorLeft.getSelectedSensorVelocity() / Constants.RPM_TO_TP100MS;
  }

  public double getRightShooterRPM() {
    return shootMotorRight.getSelectedSensorVelocity() / Constants.RPM_TO_TP100MS;
  }

  public void setAngleMotor(double speed) {
    adjustAngleMotorLeft.set(speed + Constants.FEEDFORWARD);
    adjustAngleMotorRight.set(speed + Constants.FEEDFORWARD);
  }

  public void setAngleMotorsSafe(double output) {
    if(output > Constants.SHOOT_ADJ_VOLT_LIM) output = Constants.SHOOT_ADJ_VOLT_LIM;
    if(angleEncoder.getPosition() < 1)
    {
      if(output < 0) 
      {
        setAngleMotor(0);
        return;
      } 
      else 
      {
        setAngleMotor(output);
      }
    } 
    else if(angleEncoder.getPosition() > 29)
    {
      if(output > 0) 
      {
        setAngleMotor(0);
        return;
      }
      else 
      {
        setAngleMotor(output);
      }
    } 
    else 
    {
      setAngleMotor(output);
    }
  }

  private void configShoot() {
    shootMotorLeft.configFactoryDefault();
    shootMotorRight.configFactoryDefault();

    //shootMotorRight.follow(shootMotorLeft);

    shootMotorLeft.setInverted(true);
    shootMotorRight.setInverted(false);

    shootMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMS);
    shootMotorLeft.setSensorPhase(true);

    shootMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMS);
    shootMotorRight.setSensorPhase(false);
//---------------------------------------------------------------------
    shootMotorLeft.configNominalOutputForward(0);
    shootMotorLeft.configNominalOutputReverse(0);
    shootMotorLeft.configPeakOutputForward(Constants.PEAK_SHOOTER);
    shootMotorLeft.configPeakOutputReverse(-Constants.PEAK_SHOOTER);

    shootMotorRight.configNominalOutputForward(0);
    shootMotorRight.configNominalOutputReverse(0);
    shootMotorRight.configPeakOutputForward(Constants.PEAK_SHOOTER);
    shootMotorRight.configPeakOutputReverse(-Constants.PEAK_SHOOTER);
//------------------------------------------------------------------------------
    LP = Constants.SHOOT_LEFT_VEL_P;
    shootMotorLeft.config_kF(Constants.kSlotIdx, 0, Constants.kTimeMS); //timeout maybe 30
    shootMotorLeft.config_kP(Constants.kSlotIdx, LP, Constants.kTimeMS);
    shootMotorLeft.config_kI(Constants.kSlotIdx, 0, Constants.kTimeMS);
    shootMotorLeft.config_kD(Constants.kSlotIdx, 0, Constants.kTimeMS);

    RP = Constants.SHOOT_RIGHT_VEL_P;
    shootMotorRight.config_kF(Constants.kSlotIdx, 0, Constants.kTimeMS); //timeout maybe 30
    shootMotorRight.config_kP(Constants.kSlotIdx, RP, Constants.kTimeMS);
    shootMotorRight.config_kI(Constants.kSlotIdx, 0, Constants.kTimeMS);
    shootMotorRight.config_kD(Constants.kSlotIdx, 0, Constants.kTimeMS);

    shootMotorLeft.configClosedloopRamp(.25);
    shootMotorRight.configClosedloopRamp(.25);


    adjustAngleMotorLeft.restoreFactoryDefaults();
    adjustAngleMotorLeft.setInverted(true);

    adjustAngleMotorRight.restoreFactoryDefaults();
    adjustAngleMotorRight.setInverted(false);

    shootMotorLeft.configOpenloopRamp(0.1);
    shootMotorLeft.configClosedloopRamp(0);

    angleEncoder.setPosition(0);
    angleEncoder.setPositionConversionFactor(Constants.ADJUST_ROTS_TO_DEGR);
  }
  
}
