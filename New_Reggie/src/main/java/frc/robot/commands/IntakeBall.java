/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  private final Intake intake;
  private double speed;
  double start = 0;
  double runtime = 0;


  /**
   * Creates a new IntakeBall.
   */
  public IntakeBall(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    //addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();

    intake.setIntakeMotor(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runtime = Timer.getFPGATimestamp() - start;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // sets the speed of the motor to 0 (stops) when the command ends
    intake.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(runtime > 5){
      return true;
    }
    return false;
  }
}

