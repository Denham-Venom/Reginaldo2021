// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootAndIndex extends CommandBase {
  
  Intake intake;
  Shooter shoot;
  double shooterSpeed; 
  double spinUpSpeed;
  double indexSpeed;
  long startTime;
  double seconds;
  long curTime;

  /** Creates a new ShootAndIndex. */
  public ShootAndIndex(Shooter shoot, Intake intake, double shooterSpeed, double indexSpeed, double spinUpSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    this.shooterSpeed = shooterSpeed;
    this.spinUpSpeed = spinUpSpeed;
    this.indexSpeed = indexSpeed;
    addRequirements(shoot);
  }

  public ShootAndIndex(Intake intake, Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    shooterSpeed = Constants.SHOOTER_MOTOR_SPEED;
    spinUpSpeed = Constants.SPIN_UP_SPEED;
    indexSpeed = Constants.INDEXER_SPEED;
    addRequirements(shoot);
  }

  public ShootAndIndex(Intake intake, Shooter shoot, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    this.seconds = seconds;
    shooterSpeed = Constants.SHOOTER_MOTOR_SPEED;
    spinUpSpeed = Constants.SPIN_UP_SPEED;
    indexSpeed = Constants.INDEXER_SPEED;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoot.setShooterMotors(shooterSpeed);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curTime = System.currentTimeMillis();
    if(curTime - startTime > 1000) {
      intake.setIndexerMotor(indexSpeed);
      intake.setSpinUpMotor(spinUpSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.setShooterMotors(0);
    intake.setSpinUpMotor(0);
    intake.setIndexerMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(curTime - startTime > seconds*1000) 
    //{
        //return true;
    //}
    return false;
  }
}
