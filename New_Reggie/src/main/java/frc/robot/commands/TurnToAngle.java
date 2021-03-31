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
public class TurnToAngle extends PIDCommand {
  DriveTrain dt;
  double ang;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain drivetrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DT_TURN_P, Constants.DT_TURN_I, Constants.DT_TURN_D),
        // This should return the measurement
        () -> drivetrain.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          int sign = (int) (output / Math.abs(output));
          output = output + sign * Constants.DT_TURN_F;
          drivetrain.setLeftMotors(+output);
          drivetrain.setRightMotors(-output);
        },
        drivetrain
    );
    ang = angle;
    dt = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Constants.ALLOWABLE_STEER_ERR);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();

    SmartDashboard.putNumber("turn sp", ang);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("turn err", getController().getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
    //return false;
  }
}
