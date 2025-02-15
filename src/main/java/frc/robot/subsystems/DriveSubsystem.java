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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public class pidControl {
    private TalonFX m_motor;
    private boolean m_positiveMovesForward;

    private double m_targetPosition;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        DrivetrainConstants.maxVelocity, 
        DrivetrainConstants.maxAcceleration
      )
    );

    private PositionVoltage m_positionVoltage;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setPoint = new TrapezoidProfile.State();

    public pidControl(TalonFX motor, boolean positiveMovesForward) {
      m_motor = motor;
      m_positiveMovesForward = positiveMovesForward;
    }

    public void setTargetPosition(double position) {
      m_targetPosition = position;

      double positionRotations = position * DrivetrainConstants.kDrivetrainGearRatio;
      if (!m_positiveMovesForward) {
        positionRotations = -positionRotations;
      }

      m_motor.setPosition(0); // zero's motor at the start of the movement

      m_goal = new TrapezoidProfile.State(positionRotations, 0);
      m_setPoint = new TrapezoidProfile.State(0, 0);

      m_positionVoltage = new PositionVoltage(0).withSlot(0);
    }

    public void calculatePosition() {
      m_setPoint = m_profile.calculate(0.020, m_setPoint, m_goal);

      m_positionVoltage.Position = m_setPoint.position;
      m_positionVoltage.Velocity = m_setPoint.velocity;
      m_motor.setControl(m_positionVoltage);
    }

    public double getTargetPosition() {
      return m_targetPosition;
    }

    public double getPosition() {
      double motorPosition = m_motor.getPosition().getValueAsDouble() / DrivetrainConstants.kDrivetrainGearRatio;

      if (m_positiveMovesForward) {
        return motorPosition;
      } else {
        return -motorPosition;
      }
    }
  }

  pidControl m_leftPidControl = new pidControl(
    m_leftMotor, 
    DrivetrainConstants.kLeftPositiveMovesForward
  );

  pidControl m_rightPidControl = new pidControl(
    m_rightMotor, 
    DrivetrainConstants.kRightPositiveMovesForward
  );

  //The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Setting up autoConfig
    TalonFXConfiguration autoConfig = new TalonFXConfiguration();
    autoConfig.Slot0.kV = DrivetrainConstants.kV;
    autoConfig.Slot0.kS = DrivetrainConstants.kS;
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

  // For auto, sets both motors to the target position
  public void setTargetPositionStraight(double position) {
    m_leftPidControl.setTargetPosition(position);
    m_rightPidControl.setTargetPosition(position);
  }

  // For Auto, sets the selected motors TARGET position
  public void setTargetPosition(double position, boolean isLeft) {
    if (isLeft) {
      m_leftPidControl.setTargetPosition(position);
    } else {
      m_rightPidControl.setTargetPosition(position);
    }
  }

  // For Auto, returns the selected motors TARGET position
  public double getTargetPosition(boolean isLeft) {
    if (isLeft) {
      return m_leftPidControl.getTargetPosition();
    } else {
      return m_rightPidControl.getTargetPosition();
    }
  }

  // For Auto, returns the selected motors CURRENT position
  public double getPosition(boolean isLeft)
  {
   if (isLeft) {
    return m_leftPidControl.getPosition();
   }  else {
    return m_rightPidControl.getPosition();
   }
  }

  // For Auto, returns if the left and right motor are within tolerance of the PID target
  public boolean isAtTarget(double tolerance) 
  {
    double leftError = Math.abs(m_leftPidControl.getTargetPosition() - m_leftPidControl.getPosition());
    double rightError = Math.abs(m_rightPidControl.getTargetPosition() - m_rightPidControl.getPosition());

    return (leftError <= tolerance) && (rightError <= tolerance);
  }

  // For Auto, needs to be in an auto command execute, so we can use the PID without 
  // differential drive timeing out.
  public void callDrivetrainFeed() {
    m_Drivetrain.feed();
  }

  public void callCalculate() {
    m_leftPidControl.calculatePosition();
    m_rightPidControl.calculatePosition();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}