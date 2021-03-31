// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ShootAtDistance extends CommandBase {

  private final double h1 = 1.75; //feet = 21 inches
  private final double h2 = 91/12; //feet = 91 in
  private final double a1 = 10; //degrees
  private final double a2;
  private final double d; //d = (h2-h1) / tan(a1+a2);

  private final Shooter shooter;

  /** Creates a new ShootAtDistance. */
  public ShootAtDistance(Shooter shooter) {
    a2 = Robot.llty.getDouble(0);
    d = (h2-h1) / Math.tan(Math.toRadians(a1+a2));
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterVelocity(Constants.SHOOTER_DEFAULT_VEL * d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
