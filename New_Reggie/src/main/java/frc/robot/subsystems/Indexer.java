/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel;

public class Indexer extends SubsystemBase {
  private final VictorSPX indexerMotor1 = new VictorSPX(Constants.INDEXER_MOTOR_1);
  private final VictorSPX indexerMotor2 = new VictorSPX(Constants.INDEXER_MOTOR_2);
  private final CANSparkMax spinUpMotor = new CANSparkMax(Constants.SPIN_UP_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    spinUpMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // creates SmartDashboard entries for different indexer RPMs
    // SmartDashboard.getEntry("RPM of Bells/Wheels").setDouble(getBellsRPM());
    // SmartDashboard.getEntry("RPM of Intake Wheels").setDouble(getIntakeRPM());
  }

  public void setIndexerMotor(double speed) {
    // sets the speed for the index motors
    indexerMotor1.set(ControlMode.PercentOutput, speed);
    indexerMotor2.set(ControlMode.PercentOutput, speed);
  }

  public void setSpinUpMotor(double speed) {
    // sets the speed of the spinup motor
    spinUpMotor.set(speed);
  }

public double getBellsRPM() {
  // returns to RPM of the bells
	return 0;
}

private double getIntakeRPM() {
  // returns the RPM of the intake
  return 0;
}
}
