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
  double seconds = 5;
  long curTime;
  boolean useParamVel = false;
  double target;

  /** Creates a new ShootAndIndex. */
  public ShootAndIndex(Shooter shoot, Intake intake, double shooterSpeed, double indexSpeed, double spinUpSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    this.shooterSpeed = shooterSpeed;
    this.spinUpSpeed = spinUpSpeed;
    this.indexSpeed = indexSpeed;
    addRequirements(shoot);
    useParamVel = true;
    target = shooterSpeed;
  }

  public ShootAndIndex(Intake intake, Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    shooterSpeed = Constants.SHOOTER_DEFAULT_VEL;
    spinUpSpeed = Constants.SPIN_UP_SPEED;
    indexSpeed = Constants.INDEXER_SPEED;
    addRequirements(shoot);
  }

  public ShootAndIndex(Intake intake, Shooter shoot, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    shooterSpeed = RPM;
    spinUpSpeed = Constants.SPIN_UP_SPEED;
    indexSpeed = Constants.INDEXER_SPEED;
    addRequirements(shoot);
    useParamVel = true;
    target = RPM;
  }

  public ShootAndIndex(Intake intake, Shooter shoot, double RPM, double spinUpSpeed, double indexSpeed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shoot = shoot;
    this.seconds = seconds;
    shooterSpeed = RPM;
    this.spinUpSpeed = spinUpSpeed;
    this.indexSpeed = indexSpeed;
    addRequirements(shoot);
    target = RPM;
    useParamVel = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(useParamVel) shoot.setShooterVelocity(shooterSpeed);
    else shoot.setShooterVelocity();
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curTime = System.currentTimeMillis();
    /*double ls = shoot.getLeftShooterRPM();
    double rs = shoot.getRightShooterRPM();
    if(useParamVel) target = shoot.getTargetRPM();
    if(Math.abs(ls - target) > Constants.SHOOT_ALLOWABLE_VEL_ERR 
        || Math.abs(rs - target) > Constants.SHOOT_ALLOWABLE_VEL_ERR 
        || Math.abs(ls - rs) > Constants.SHOOT_ALLOWABLE_VEL_ERR) {
      intake.setIndexerMotor(0);
      intake.setSpinUpMotor(0);
    } else*/ if(curTime - startTime > 1500) {
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
    if(curTime - startTime > seconds*1000) {
        return true;
    }
    return false;
  }
}
