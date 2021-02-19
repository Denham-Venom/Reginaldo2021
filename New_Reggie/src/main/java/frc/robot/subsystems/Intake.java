/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  public final DoubleSolenoid intakeDoubleSolenoid = new DoubleSolenoid(0, 1);
  private boolean extended = false;
  /**
   * Creates a new Intake.
   */
  
  public Intake() {
    intakeDoubleSolenoid.set(Value.kReverse);
  }

  public void toggleIntake() {
    if(extended) {
      intakeDoubleSolenoid.set(Value.kReverse);
      extended = false;
    } else {
      intakeDoubleSolenoid.set(Value.kForward);
      extended = true;
    }
  }
  
  public void extendIntake() {
    intakeDoubleSolenoid.set(Value.kForward);
  }

  public void releaseIntake() {
    intakeDoubleSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeMotor(double speed) {
    // sets the intake motor to the given speed
      intakeMotor.set(speed);
  }

public double getIntakeRPM() {
	return 0;
}
}