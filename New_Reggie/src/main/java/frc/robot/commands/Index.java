/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {
  Indexer indexer;
  /**
   * Creates a new Index.
   */
  public Index(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // runs the index and spin up motors so the balls can reach the shooter
    indexer.setIndexerMotor(Constants.INDEXER_SPEED);
    indexer.setSpinUpMotor(Constants.SPIN_UP_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // sets the speed of the speed of the two motors to 0 (stops) when the command ends
    indexer.setIndexerMotor(0);
    indexer.setSpinUpMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

