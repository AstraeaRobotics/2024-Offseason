// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.auto.components.AutoShoot;
// import frc.robot.commands.auto.components.DriveToDistance;
// import frc.robot.commands.auto.components.TurnWheels;
import frc.robot.commands.pivot.SetState;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.Constants.ShooterConstants.ShooterStates;
 

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAuto extends SequentialCommandGroup {
  /** Creates a new OneNoteAuto. */
  public OneNoteAuto(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
      // new TurnWheels(swerveSubsystem, 0, -1, 0),
      new AutoShoot(shooterSubsystem, ShooterStates.kSpeaker), 
      new SetState(shooterSubsystem, ShooterStates.kGround)
      // new DriveToDistance(swerveSubsystem, 0, -1)
    );
  }
}
