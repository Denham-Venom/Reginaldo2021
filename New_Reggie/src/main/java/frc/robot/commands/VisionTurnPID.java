// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionTurnPID extends PIDCommand {

  private final DriveTrain dt;
  private static double steerF = Robot.steerF.getDouble(0);
  private long start;
  private long cur;

  /** Creates a new VisionTurnPID. */
  public VisionTurnPID(DriveTrain dt) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DT_TURN_P, Constants.DT_TURN_I, Constants.DT_TURN_D),
        // This should return the measurement
        () -> (Robot.lltv.getDouble(0) == 1) ? Robot.lltx.getDouble(0) : 27, // tries to turn 27 degrees if no target found, tv = 1 means target found
        // This should return the setpoint (can also be a constant)
        () -> 0, // 0 represents robot being centered on target
        // This uses the output
        output -> {
          double err = Robot.lltx.getDouble(1);
          int sign = (int) (err / Math.abs(err));
          output = output + sign * Constants.DT_TURN_F; //TODO-maybe minus, idk
          dt.setLeftMotors(-output);
          dt.setRightMotors(+output);
      }
    );
    this.dt = dt;
    addRequirements(dt);
    getController().setTolerance(Constants.ALLOWABLE_STEER_ERR);
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    Robot.ledMode.setDouble(0);
  }

  // @Override
  // public void execute() {
  //   super.execute();
  //   getController().setP(Robot.steerP.getDouble(0));
  //   getController().setI(Robot.steerI.getDouble(0));
  //   getController().setD(Robot.steerD.getDouble(0));
  //   steerF = Robot.steerF.getDouble(0);
  // }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Robot.ledMode.setDouble(1);
    dt.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
