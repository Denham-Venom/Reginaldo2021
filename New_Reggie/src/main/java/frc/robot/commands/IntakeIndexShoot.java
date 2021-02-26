// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BDSM license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeIndexShoot extends CommandBase {
  /** Creates a new IntakeIndexShoot. */
  Intake intake;
  double intakeSpeed;
  double spinUpSpeed;
  double indexSpeed;
  public IntakeIndexShoot(Intake intake, double intakeSpeed, double spinUpSpeed, double indexSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.spinUpSpeed = spinUpSpeed;
    this.indexSpeed = indexSpeed;
    addRequirements(intake);
  }

  public IntakeIndexShoot(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    intakeSpeed = Constants.INTAKE_BALL_SPEED;
    spinUpSpeed = Constants.SPIN_UP_SPEED;
    indexSpeed = Constants.INDEXER_SPEED;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeMotor(intakeSpeed);
    intake.setIndexerMotor(indexSpeed);
    intake.setSpinUpMotor(spinUpSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMotor(0);
    intake.setIntakeMotor(0);
    intake.setSpinUpMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
