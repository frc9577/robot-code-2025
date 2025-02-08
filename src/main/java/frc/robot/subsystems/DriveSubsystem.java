// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment. What does this class do and what assumptions
// (if any) is it making?

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.TankDriveCommand;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
  private TalonFX m_optionalRightMotor;

  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  //private final LaserCan m_laserCan = new LaserCan(DrivetrainConstants.kLaserCanCANID);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Right Motor Setup
    final MotorOutputConfigs m_rightMotorOutputConfigs = new MotorOutputConfigs();
    m_rightMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // why is it a enum :sob:
    m_rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator m_rightMotorConfigurator = m_rightMotor.getConfigurator();
    m_rightMotorConfigurator.apply(m_rightMotorOutputConfigs);
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Optional Right Motor
    try {
      m_optionalRightMotor = new TalonFX(DrivetrainConstants.kOptionalRightMotorCANID);

      // Setting up Config
      final MotorOutputConfigs m_rightOptionalMotorOutputConfigs = new MotorOutputConfigs();
      m_rightOptionalMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      m_rightOptionalMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving Config
      final TalonFXConfigurator m_rightOptionalMotorConfigurator = m_optionalRightMotor.getConfigurator();
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
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Optional Left Motor
    try {
      m_optionalLeftMotor = new TalonFX(DrivetrainConstants.kOptionalLeftMotorCANID);

      // Setting up config
      final MotorOutputConfigs m_leftOptionalMotorOutputConfigs = new MotorOutputConfigs();
      m_leftOptionalMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      m_leftOptionalMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving config
      final TalonFXConfigurator m_leftOptionalMotorConfigurator = m_optionalLeftMotor.getConfigurator();
      m_leftOptionalMotorConfigurator.apply(m_leftOptionalMotorOutputConfigs);

      // Setting as follower
      m_optionalLeftMotor.setControl(
        new Follower(m_leftMotor.getDeviceID(), false)
      );
    }
    catch (Exception e)
    {
      // TODO: There are really two cases you want to catch. The first, when the follower
      // motor controller doesn't exist, isn't an error. The second, where the motor exists
      // but one of the later configuration calls fails, is an error. Generally, you would
      // only dump a stack trace in error cases and you definitely don't want to do this in
      // normal operation. I would suggest splitting this block into two try/except chunks,
      // on that catches the missing controller and just outputs a status message to the log
      // indicating that only one motor is in use, and the other catching the real errors
      // and dumping the stack trace.
      e.printStackTrace();
    }

    // Setting up the drive train
    m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    SendableRegistry.setName(m_Drivetrain, "DriveSubsystem", "Drivetrain");    
  }

  public void initDefaultCommand(Joystick Joystick, XboxController Controller, boolean isArcade)
  {
    if (isArcade == true) {
      setDefaultCommand(new ArcadeDriveCommand(this, Joystick));
    } else {
      setDefaultCommand(new TankDriveCommand(this, Controller));
    }
  }

  // Sets left and right motors to set speeds to support tank drive models.
  public void setTankSpeeds(double leftInput, double rightInput)
  {
    m_leftSpeed = (leftInput / DrivetrainConstants.kSpeedDivider);
    m_rightSpeed = (rightInput / DrivetrainConstants.kSpeedDivider);

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.tankDrive(m_leftSpeed, m_rightSpeed, true);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
