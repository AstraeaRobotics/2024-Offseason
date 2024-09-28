// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootTrap extends SequentialCommandGroup {
  /** Creates a new ShootTrap. */
  public ShootTrap(ShooterSubsystem m_shoot) {
    addCommands(new ParallelDeadlineGroup(new WaitCommand(.5), new PreShoot(m_shoot)), new Shoot(m_shoot, ShooterConstants.kTrapSpeed));
  }

}
