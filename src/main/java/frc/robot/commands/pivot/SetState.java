// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.util.LimelightUtil;
import frc.robot.Constants.ShooterConstants.ShooterStates;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetState extends InstantCommand {

  private ShooterSubsystem m_shooterSubsystem;
  private ShooterStates m_state;

  public SetState(ShooterSubsystem shooterSubsystem, ShooterStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_state = state;

    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_shooterSubsystem.setState(m_state);
    m_shooterSubsystem.setDesiredPosition(m_state.getPivotSetpoint());
    m_shooterSubsystem.setManualOffset(0);
    //LimelightUtil.setPipeline(m_shooterSubsystem.getState());
  }
}