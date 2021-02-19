// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  private final DriveTrain drive;
  private final Joystick pilot;
  DriveTrain getInverted;
  double[] arr;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain drive, Joystick pilotDriverController) {
    this.drive = drive;
    pilot = pilotDriverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftStickY = pilot.getRawAxis(Constants.LY_AXIS);
    double rightStickX = pilot.getRawAxis(Constants.RX_AXIS);

    if(drive.getInverted()) {
      leftStickY *= -1;
    }

    arr = drive.highLowGearValue();
    leftStickY *= arr[0];
    rightStickX *= arr[1];

    drive.setLeftMotors(leftStickY + rightStickX);
    drive.setRightMotors(leftStickY - rightStickX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
