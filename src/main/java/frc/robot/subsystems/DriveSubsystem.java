// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
  private DifferentialDrive m_Drivetrain;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() { 
    // Right Motor
    final MotorOutputConfigs m_rightMotorOutputConfigs = new MotorOutputConfigs();
    m_rightMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // why is it a enum :sob:
    m_rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator m_rightMotorConfigurator = m_rightMotor.getConfigurator();
    m_rightMotorConfigurator.apply(m_rightMotorOutputConfigs);

    // Left Motor
    final MotorOutputConfigs m_leftMotorOutputConfigs = new MotorOutputConfigs();
    m_leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator m_leftMotorConfigurator = m_leftMotor.getConfigurator();
    m_leftMotorConfigurator.apply(m_leftMotorOutputConfigs);

    // TODO: Figure out how to make a differentialDrive w/ these motors
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
