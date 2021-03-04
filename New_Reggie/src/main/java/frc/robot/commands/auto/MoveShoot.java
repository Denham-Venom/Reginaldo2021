// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Move;
import frc.robot.commands.ShootAndIndex;
import frc.robot.commands.VisionAim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveShoot extends SequentialCommandGroup {
  /** Creates a new MoveShoot. */
  public MoveShoot(DriveTrain dt, Shooter s, Intake i, double shootTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> i.extendIntake()),
      new Move(dt, 15), 
      new VisionAim(dt, s),
      new ShootAndIndex(i, s, shootTime)
      );
  }
}
