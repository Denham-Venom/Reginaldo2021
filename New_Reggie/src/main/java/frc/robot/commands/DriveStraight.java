// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends PIDCommand {

  private final DriveTrain dt;
  private final double feet;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrain dt, double feet, double startAngle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DT_PID_FT_P, 0, 0),
        // This should return the measurement
        () -> dt.getAverageEncoderDistanceFeet(),
        // This should return the setpoint (can also be a constant)
        dt.getAverageEncoderDistanceFeet() + feet,
        // This uses the output
        output -> {
          double ang = dt.getAngle();
          double diff = ang - startAngle;
          if(diff > Constants.ALLOWABLE_AIM_ERR) {
            int sign = (int) Math.round(Math.abs(diff) / diff);
            double turn = (diff) * Constants.DT_TURN_P + sign * Constants.DT_TURN_F;
            dt.setLeftMotors(output - turn);
            dt.setRightMotors(output + turn);
          } else {
            dt.setLeftMotors(output);
            dt.setRightMotors(output);
          }
        });
    addRequirements(dt);
    this.dt = dt;
    this.feet = feet;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.allowableCloseLoopError);
    setName("DriveStraight");
    setSubsystem("DriveTrain");
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    SmartDashboard.putNumber("goin", dt.getAverageEncoderDistanceFeet() + feet);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("cur", dt.getAverageEncoderDistanceFeet());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(getController().atSetpoint()) return true;
    return false;
  }
}
