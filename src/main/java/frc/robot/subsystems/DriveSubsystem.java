// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment. What does this class do and what assumptions
// (if any) is it making?

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
  private TalonFX m_optionalRightMotor;

  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private TalonFX m_optionalLeftMotor; 

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  private final PositionVoltage m_rightPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionVoltage m_leftPositionVoltage = new PositionVoltage(0).withSlot(0);

  private double m_leftTargetPosition = 0.0;
  private double m_rightTargetPosition = 0.0;

  //The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Setting up autoConfig
    TalonFXConfiguration autoConfig = new TalonFXConfiguration();
    autoConfig.Slot0.kP = DrivetrainConstants.kP;
    autoConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    autoConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

    autoConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.PeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    // Right Motor Setup
    final MotorOutputConfigs rightMotorOutputConfigs = new MotorOutputConfigs();
    rightMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // why is it a enum :sob:
    rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator rightMotorConfigurator = m_rightMotor.getConfigurator();
    rightMotorConfigurator.apply(rightMotorOutputConfigs);
    rightMotorConfigurator.apply(autoConfig); // i hope it wont overwrite above config
    
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Optional Right Motor
    try {
      m_optionalRightMotor = new TalonFX(DrivetrainConstants.kOptionalRightMotorCANID);

      // Setting up Config
      final MotorOutputConfigs rightOptionalMotorOutputConfigs = new MotorOutputConfigs();
      rightOptionalMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      rightOptionalMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving Config
      final TalonFXConfigurator rightOptionalMotorConfigurator = m_optionalRightMotor.getConfigurator();
      rightOptionalMotorConfigurator.apply(rightOptionalMotorOutputConfigs);
      rightOptionalMotorConfigurator.apply(autoConfig); // i hope it wont overwrite above config

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
    final MotorOutputConfigs leftMotorOutputConfigs = new MotorOutputConfigs();
    leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    final TalonFXConfigurator leftMotorConfigurator = m_leftMotor.getConfigurator();
    leftMotorConfigurator.apply(leftMotorOutputConfigs);
    leftMotorConfigurator.apply(autoConfig); // i hope it wont overwrite above config
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Optional Left Motor
    try {
      m_optionalLeftMotor = new TalonFX(DrivetrainConstants.kOptionalLeftMotorCANID);

      // Setting up config
      final MotorOutputConfigs leftOptionalMotorOutputConfigs = new MotorOutputConfigs();
      leftOptionalMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
      leftOptionalMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
  
      // Saving config
      final TalonFXConfigurator leftOptionalMotorConfigurator = m_optionalLeftMotor.getConfigurator();
      leftOptionalMotorConfigurator.apply(leftOptionalMotorOutputConfigs);
      leftOptionalMotorConfigurator.apply(autoConfig); // i hope it wont overwrite above config

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

    // Zeroing the encoders
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    // Setting up the drive train
    //m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    //SendableRegistry.setName(m_Drivetrain, "DriveSubsystem", "Drivetrain");  
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
    //m_Drivetrain.tankDrive(m_leftSpeed, m_rightSpeed, true);
  }

  public void setArcadeSpeeds(double speed, double rotation)
  {
    m_leftSpeed = (speed / DrivetrainConstants.kSpeedDivider);
    m_rightSpeed = rotation;

    // NOTE: We are squaring the input to improve driver response
    //m_Drivetrain.arcadeDrive(m_leftSpeed, m_rightSpeed, true);
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

  // For Auto, sets the left motors TARGET position
  public void setLeftTargetPosition(double position)
  {
    m_leftTargetPosition = position;

    double leftPositionRotations = position * DrivetrainConstants.kDrivetrainGearRatio;
    if (!DrivetrainConstants.kLeftPositiveMovesForward) {
      leftPositionRotations = -leftPositionRotations;
    }

    System.out.println("LPR: " + leftPositionRotations);

    m_leftMotor.setPosition(0); // zero's motor at the start of the movement
    m_leftMotor.setControl(m_leftPositionVoltage.withPosition(leftPositionRotations));
  }

  // For Auto, returns the left motors TARGET position
  public double getLeftTargetPosition()
  {
    return m_leftTargetPosition;
  }

  // For Auto, returns the left motors CURRENT position
  public double getLeftPosition()
  {
    double leftMotorPosition = m_leftMotor.getPosition().getValueAsDouble() / DrivetrainConstants.kDrivetrainGearRatio;

    if (DrivetrainConstants.kLeftPositiveMovesForward) {
      return leftMotorPosition;
    } else {
      return -leftMotorPosition;
    }
  }

  // For Auto, sets the right motors TARGET position
  public void setRightTargetPosition(double position)
  {
    m_rightTargetPosition = position;

    double rightPositionRotations = position * DrivetrainConstants.kDrivetrainGearRatio;
    if (!DrivetrainConstants.kRightPositiveMovesForward) {
      rightPositionRotations = -rightPositionRotations;
    }

    System.out.println("RPR:" + rightPositionRotations);

    m_rightMotor.setPosition(0); // zero's motor at the start of the movement
    m_rightMotor.setControl(m_rightPositionVoltage.withPosition(rightPositionRotations));
  }

  // For Auto, returns the right motors TARGET position
  public double getRightTargetPosition()
  {
    return m_rightTargetPosition;
  }

  // For Auto, returns the right motors CURRENT position
  public double getRightPosition()
  {
    double rightMotorPosition = m_rightMotor.getPosition().getValueAsDouble() / DrivetrainConstants.kDrivetrainGearRatio;

    if (DrivetrainConstants.kRightPositiveMovesForward) {
      return rightMotorPosition;
    } else {
      return -rightMotorPosition;
    }
  }

  // For Auto, returns if the left and right motor are within tolerance of the PID target
  public boolean isAtTarget(double tolerance) 
  {
    double leftError = Math.abs(m_leftTargetPosition - getLeftPosition());
    double rightError = Math.abs(m_rightTargetPosition - getRightPosition());

    return (leftError <= tolerance) && (rightError <= tolerance);
  }

  // For Auto, needs to be in an auto command execute, so we can use the PID without 
  // differential drive timeing out.
  public void callDrivetrainFeed() {
    //m_Drivetrain.feed();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void report() {
    System.out.println("RMV: " + m_rightMotor.getVelocity());
  }
}
