// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
  private TalonFX m_optionalRightMotor;

  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Right Motor Setup
    final MotorOutputConfigs m_rightMotorOutputConfigs = new MotorOutputConfigs();
    m_rightMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // why is it a enum :sob:
    m_rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator m_rightMotorConfigurator = m_rightMotor.getConfigurator();
    m_rightMotorConfigurator.apply(m_rightMotorOutputConfigs);

    // Optional Right Motor
    try {
      m_optionalRightMotor = new TalonFX(DrivetrainConstants.kOptionalRightMotorCANID);

      // Setting up Config
      final MotorOutputConfigs m_rightOptionalMotorOutputConfigs = new MotorOutputConfigs();
      m_rightMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      m_rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving Config
      final TalonFXConfigurator m_rightOptionalMotorConfigurator = m_rightMotor.getConfigurator();
      m_rightOptionalMotorConfigurator.apply(m_rightOptionalMotorOutputConfigs);

      // Setting as a follwer
      m_optionalRightMotor.setControl(
        new Follower(m_rightMotor.getDeviceID(), false)
      );
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }

    // Left Motor Setup
    final MotorOutputConfigs m_leftMotorOutputConfigs = new MotorOutputConfigs();
    m_leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator m_leftMotorConfigurator = m_leftMotor.getConfigurator();
    m_leftMotorConfigurator.apply(m_leftMotorOutputConfigs);

    // Optional Left Motor
    try {
      m_optionalLeftMotor = new TalonFX(DrivetrainConstants.kOptionalLeftMotorCANID);

      // Setting up config
      final MotorOutputConfigs m_leftOptionalMotorOutputConfigs = new MotorOutputConfigs();
      m_leftMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      m_leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving config
      final TalonFXConfigurator m_rightOptionalMotorConfigurator = m_rightMotor.getConfigurator();
      m_rightOptionalMotorConfigurator.apply(m_leftOptionalMotorOutputConfigs);

      // Setting as follower
      m_optionalLeftMotor.setControl(
        new Follower(m_rightMotor.getDeviceID(), false)
      );
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }

    // Setting up the drive train
    m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  }

  public void initDefaultCommand(Joystick leftJoystick)
  {
    setDefaultCommand(new ArcadeDriveCommand(this, leftJoystick));
  }

  public void setArcadeSpeeds(double speed, double rotation)
  {
    m_leftSpeed = (speed / DrivetrainConstants.kSpeedDivider);
    m_rightSpeed = rotation;

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.arcadeDrive(m_leftSpeed, m_rightSpeed, true);
  }

  public double getSpeed(boolean bLeft)
  {
    if (bLeft)
    {
      return m_leftSpeed;
    }
    else
    {
      return m_rightSpeed;
    }
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
