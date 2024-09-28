
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeSubsystem extends SubsystemBase {
  public static CANSparkMax preIntakeMotor;
  public static CANSparkMax intakeMotor;

  public static RelativeEncoder intakeEncoder;
  public static RelativeEncoder preIntakeEncoder;

  public static SimpleMotorFeedforward intakeFeedForward;
  public static SimpleMotorFeedforward preIntakeFeedForward;

  public IntakeSubsystem() {
    preIntakeMotor = new CANSparkMax(10, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(9, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    preIntakeEncoder = preIntakeMotor.getEncoder();

    intakeFeedForward = new SimpleMotorFeedforward(IntakeConstants.intakeKS, IntakeConstants.intakeKV);
    preIntakeFeedForward = new SimpleMotorFeedforward(IntakeConstants.preIntakeKS, IntakeConstants.preIntakeKV);

    configureMotors();

    intakeEncoder.setVelocityConversionFactor(.0167); // 1 / 60
    preIntakeEncoder.setVelocityConversionFactor(.0167);
  }

  public void spinPreIntake(double speed){
    preIntakeMotor.set(speed);
  }

  public void spinIntake(double speed){
    intakeMotor.set(speed);
  }

  public void configureMotors() {
    preIntakeMotor.setSmartCurrentLimit(35);
    preIntakeMotor.setInverted(true);
    preIntakeMotor.setIdleMode(IdleMode.kCoast);
    preIntakeMotor.burnFlash();

    intakeMotor.setSmartCurrentLimit(35);
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  public void setVoltageInt(double volts) {
    intakeMotor.setVoltage(volts);
  }

  public void setVoltagePreInt(double volts){
    preIntakeMotor.setVoltage(volts);
  }

  public void setIntake(double speed) {
    intakeMotor.setVoltage(intakeFeedForward.calculate(speed));
  }

  public void setPreIntake(double speed){
    preIntakeMotor.setVoltage(preIntakeFeedForward.calculate(speed));
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getPreIntakeVelocity(){
    return preIntakeEncoder.getVelocity();
  }

  public double getCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
