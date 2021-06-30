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
public class TrenchRun extends SequentialCommandGroup {
  /** Creates a new TrenchRun. Back up, shoot, turn and move toward trench, collect all balls under trench
   * go back to starting point, back up, shoot
   */
  public TrenchRun(DriveTrain dt, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> intake.extendIntake()),
      new AimAndShoot(dt, shooter, intake),
      new TurnToAngle(dt, -90.0), 
      new Move(dt, -5.16),  //hopefully goes backwards 5 feet 2 inches
      new TurnToAngle(dt, 90.0),
      new MoveAndIntake(dt, intake, -24.0),  //24 feet
      new Move(dt, 24.0),
      new AimAndShoot(dt, shooter, intake)
      
      //if starting right in front of goal******
      // aim and shoot *
      // turn 90 counter-clockwise *
      // go backwards 5 feet *
      // turn clockwise 90 *
      // go backwards while running intake 24 feet *
      // go forward 24 feet *
      // aim and shoot *
    );
  }
}
