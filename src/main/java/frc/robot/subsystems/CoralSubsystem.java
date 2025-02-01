// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment explaining what this class manages. For
// example, does it manage only the coral handler on the elevator or does
// it include intake mechanism too?

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
  private double m_intakeSpeed = 0;
  private double m_outputSpeed = 0;

  private final SparkMax m_intakeMotor = new SparkMax(CoralConstants.kIntakeMotorCANID, 
                                                      MotorType.kBrushless);
  private final SparkMax m_outputMotor = new SparkMax(CoralConstants.kOutputMotorCANID,
                                                      MotorType.kBrushless);

  private final DigitalInput m_FrontSensor = new DigitalInput(CoralConstants.kFrontSensorChannel);
  private final DigitalInput m_BackSensor = new DigitalInput(CoralConstants.kBackSensorChannel);

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    // We need to know if the motor controllers we need are
    // actually present on the CAN bus and, unfortunately, their 
    // constructors don't see to throw exceptions in this case. Let's
    // ask for the encoder handle and see if it's valid.
    if (m_intakeMotor.getFaults().can || m_outputMotor.getFaults().can)
    {
      throw new RuntimeException("Coral subsystem motors not present");
    }
  }

  public void setIntakeSpeed(double speed)
  {
    m_intakeMotor.set(speed);
    m_intakeSpeed = speed;
  }
  
  // Returns the last COMMANDED speed
  public double getIntakeSpeed()
  {
    return m_intakeSpeed;
  }

  public void setOutputSpeed(double speed)
  {
    m_outputMotor.set(speed);
    m_outputSpeed = speed;
  }

  // Returns the last COMMANDED speed.
  public double getOutputSpeed()
  {
    return m_outputSpeed;
  }

  public boolean detectsCoralAtFront()
  {
    boolean sensorRead = m_FrontSensor.get();
    return CoralConstants.kFrontSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }
  
  public boolean detectsCoralAtBack()
  {
    boolean sensorRead = m_BackSensor.get();
    return CoralConstants.kBackSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  public boolean hasCoral()
  {
    return (detectsCoralAtFront() || detectsCoralAtBack());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: You may want to subdivide this so that it doesn't update too quickly.
    // We may also want to centralize all the updates an call a function from the
    // top level robot periodic() function instead of having each subsystem send
    // their own data. That high level function would poll subsystems for status
    // and send all data from a single place.
    SmartDashboard.putBoolean("Has Coral", hasCoral());
    SmartDashboard.putBoolean("Coral Front", detectsCoralAtFront());
    SmartDashboard.putBoolean("Coral Back", detectsCoralAtBack());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
