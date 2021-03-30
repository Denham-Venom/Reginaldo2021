// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class Move extends CommandBase {
  DriveTrain dt;
  double feet;
  double start = 0;

  /** Creates a new Move. */
  public Move(DriveTrain dt, double feet) {
    this.dt = dt;
    this.feet = feet;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feet = Robot.move.getDouble(0);
    dt.setWithPostion(feet);
    start = dt.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("amount", dt.getPosition() / Constants.FEET_TO_ROT_UNITS - start);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(dt.getClosedLoopErrorFeet() < Constants.DT_MOVE_ERR) return true;
    return false;
  }
}
