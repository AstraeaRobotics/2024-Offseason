// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants.ShooterStates;
import frc.robot.commands.auto.paths.OneNoteAuto;
import frc.robot.commands.auto.paths.PathPlannerTest;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.pivot.ManualRotation;
import frc.robot.commands.pivot.SetState;
import frc.robot.commands.shooter.PostIntake;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.shooter.ShooterIntake;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.ResetPose;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.vision.AlignAndPivot;
// import frc.robot.commands.vision.AlignToShoot;
import frc.robot.commands.vision.PivotAlign;
import frc.robot.commands.vision.RotationalAlign;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final PS4Controller m_driverController = new PS4Controller(0);
  public static final GenericHID operatorGamepad = new GenericHID(1);

  public static final JoystickButton kOperator1 = new JoystickButton(operatorGamepad, 1);
  public static final JoystickButton kOperator2 = new JoystickButton(operatorGamepad, 2);
  public static final JoystickButton kOperator3 = new JoystickButton(operatorGamepad, 3);
  public static final JoystickButton kOperator4 = new JoystickButton(operatorGamepad, 4);
  public static final JoystickButton kOperator5 = new JoystickButton(operatorGamepad, 5);
  public static final JoystickButton kOperator6 = new JoystickButton(operatorGamepad, 6);
  public static final JoystickButton kOperator7 = new JoystickButton(operatorGamepad, 7);
  public static final JoystickButton kOperator8 = new JoystickButton(operatorGamepad, 8);
  public static final JoystickButton kOperator9 = new JoystickButton(operatorGamepad, 9);
  public static final JoystickButton kOperator10 = new JoystickButton(operatorGamepad, 10);
  public static final JoystickButton kOperator11 = new JoystickButton(operatorGamepad, 11);
  public static final JoystickButton kOperator12 = new JoystickButton(operatorGamepad, 12);

  private final JoystickButton kCross = new JoystickButton(m_driverController, PS4Controller.Button.kCross.value);
  private final JoystickButton kCircle = new JoystickButton(m_driverController, PS4Controller.Button.kCircle.value);
  private final JoystickButton kSquare = new JoystickButton(m_driverController, PS4Controller.Button.kSquare.value);
  private final JoystickButton kTriangle = new JoystickButton(m_driverController, PS4Controller.Button.kTriangle.value);
  private final JoystickButton kL1 = new JoystickButton(m_driverController, PS4Controller.Button.kL1.value);
  private final JoystickButton kR1 = new JoystickButton(m_driverController, PS4Controller.Button.kR1.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_SwerveSubsystem.setDefaultCommand(new TeleopSwerve(m_SwerveSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    kCross.onTrue(new ResetGyro(m_SwerveSubsystem));
    kCircle.onTrue(new PathPlannerTest(m_SwerveSubsystem));

    // kOperator1.whileTrue(new ShooterIntake(m_shooterSubsystem)); // INT
    // kOperator1.whileTrue(new ParallelCommandGroup(new IntakeNote(m_intakeSubsystem, 70, 70), new ShooterIntake(m_shooterSubsystem)));
    // kOperator2.whileTrue(new ShootNote(m_shooterSubsystem)); // SHT
    // kOperator3.whileTrue(new ShootAmp(m_shooterSubsystem)); // AST
    // kOperator4.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kGround)); // GRD
    // kOperator5.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kSpeaker)); // SPK
    // kOperator6.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kAmp)); // AMP
    // kOperator7.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kSpeaker2)); // SID
    // kOperator8.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kFeed));
    // kOperator9.whileTrue(new PostIntake(m_shooterSubsystem)); // PI
    // kOperator10.onTrue(new ManualRotation(m_shooterSubsystem, true));
    // kOperator11.onTrue(new ManualRotation(m_shooterSubsystem, false)); // RST
    // kOperator12.onTrue(new SetState(m_shooterSubsystem, ShooterStates.kSource));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new PathPlannerAuto("Line");
    return null;
  }
}