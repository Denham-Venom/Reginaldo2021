// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  double speed;
  private final TalonFX shootMotorLeft = new TalonFX(Constants.SHOOT_MOTOR_LEFT_ID);
  private final TalonFX shootMotorRight = new TalonFX(Constants.SHOOT_MOTOR_RIGHT_ID);
  private final CANSparkMax adjustAngleMotorLeft = new CANSparkMax(Constants.ADJUST_ANGLE_MOTOR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax adjustAngleMotorRight = new CANSparkMax(Constants.ADJUST_ANGLE_MOTOR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final CANEncoder angleEncoder = adjustAngleMotorLeft.getEncoder();
//---------------------------------------------------------------
  private static ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
  static final NetworkTableEntry shooterSpeed = tuning.add("Adjust shooter speed", 0).getEntry(); 






  public Shooter() {
    configShoot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //ff = SmartDashboard.getEntry("Feedforward Shooter").getDouble(0);
    
    SmartDashboard.putNumber("Shooter Angle", angleEncoder.getPosition());
    speed = shooterSpeed.getDouble(0);
  }

  public void setShooterMotors(double speed) {
    shootMotorLeft.set(ControlMode.PercentOutput, speed);
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

    shootMotorRight.follow(shootMotorLeft);

    shootMotorLeft.setInverted(true);
    shootMotorRight.setInverted(TalonFXInvertType.OpposeMaster);

    shootMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMS);

    shootMotorLeft.setSensorPhase(true);

    shootMotorLeft.configNominalOutputForward(0);
    shootMotorLeft.configNominalOutputReverse(0);
    shootMotorLeft.configPeakOutputForward(1);
    shootMotorLeft.configPeakOutputReverse(-1);

    shootMotorLeft.config_kF(Constants.kSlotIdx, 0, Constants.kTimeMS); //timeout maybe 30
    shootMotorLeft.config_kP(Constants.kSlotIdx, 0, Constants.kTimeMS);
    shootMotorLeft.config_kI(Constants.kSlotIdx, 0, Constants.kTimeMS);
    shootMotorLeft.config_kD(Constants.kSlotIdx, 0, Constants.kTimeMS);

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
