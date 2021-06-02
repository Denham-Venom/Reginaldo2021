// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterAngle extends PIDCommand {

  private final Shooter s;

  /** Creates a new SetShooterAngle. */
  public SetShooterAngle(Shooter shooter, double angle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.SHOOT_AIM_P, Constants.SHOOT_AIM_I, Constants.SHOOT_AIM_D),
        // This should return the measurement
        () -> shooter.angleEncoder.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        output -> {
          int sign = (int) (output / Math.abs(output));
          output = output + sign * Constants.SHOOT_AIM_F;
          shooter.setAngleMotorsSafe(output);
        });
    s = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.ALLOWABLE_AIM_ERR);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Robot.ledMode.setDouble(1);
    s.setAngleMotorsSafe(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
