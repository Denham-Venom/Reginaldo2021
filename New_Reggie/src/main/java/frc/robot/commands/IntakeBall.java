/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  private final Intake intake;
  private double speed;
  private double start = 0;
  private double runtime = 0;
  private final double time;


  /**
   * Creates a new IntakeBall.
   */
  public IntakeBall(Intake intake, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    this.time = time;
    //addRequirements(intake);
  }

  /**
   * Creates a new IntakeBall.
   */
  public IntakeBall(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = Constants.INTAKE_BALL_SPEED;
    this.time = Constants.INTAKE_DEFAULT_TIME;
    //addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //start = Timer.getFPGATimestamp();
    start = System.currentTimeMillis();

    intake.setIntakeMotor(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //runtime = Timer.getFPGATimestamp() - start;
    runtime = System.currentTimeMillis() - start; 
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
    if(runtime > 5000){
      return true;
    }
    return false;
  }
}

