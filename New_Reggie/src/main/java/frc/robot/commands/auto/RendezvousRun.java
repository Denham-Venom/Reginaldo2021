// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.Move;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RendezvousRun extends SequentialCommandGroup {
  /** Creates a new RendezvousRun. */
  public RendezvousRun(DriveTrain dt, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> intake.extendIntake()), // extends intake
      new AimAndShoot(dt, shooter, intake), // aims and shoots
      new TurnToAngle(dt, -90), // turn 90 degrees left 
      new MoveAndIntake(dt, intake, -18), // moves 18 feet while running intake
      new TurnToAngle(dt, 45), // turns 45 degrees
      new Move(dt, 18), // moves 18 feet
      new AimAndShoot(dt, shooter, intake) // aims and shoots
    
      //Aim and shoot *
      //Rotate 90 degrees (counter- clockwise) *
      //Move backwards (approx. -18 feet) *
      //Rotate 45 degrees (clockwise) *
      //Move backwards (approx. 18 feet) *
      //Aim and shoot *
    );
  }
}
