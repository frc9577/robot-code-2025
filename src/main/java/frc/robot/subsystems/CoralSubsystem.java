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

  //private final SparkMax m_intakeMotor = new SparkMax(CoralConstants.kIntakeMotorCANID, 
  //                                                          MotorType.kBrushless);
  //private final SparkMax m_outputMotor = new SparkMax(CoralConstants.kOutputMotorCANID,
  //                                                          MotorType.kBrushless);

  // TODO: Which sensor? I assume there will need to be several.
  private final DigitalInput m_Sensor = new DigitalInput(CoralConstants.kSensorChannel);

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {}

  public void setIntakeSpeed(double speed)
  {
    //m_intakeMotor.set(speed);
    m_intakeSpeed = speed;
  }
  
  // Returns the last COMMANDED speed
  public double getIntakeSpeed()
  {
    return m_intakeSpeed;
  }

  public void setOutputSpeed(double speed)
  {
    //m_outputMotor.set(speed);
    m_outputSpeed = speed;
  }

  // Returns the last COMMANDED speed.
  public double getOutputSpeed()
  {
    return m_outputSpeed;
  }

  public boolean hasCoral()
  {
    boolean sensorRead = m_Sensor.get();
    return CoralConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO: You may want to subdivide this so tht it doesn't update too quickly.
    // We may also want to centralize all the updates an call a function from the
    // top level robot periodic() function instead of having each subsystem send
    // their own data. That high level function would poll subsystems for status
    // and send all data from a single place.
    SmartDashboard.putBoolean("Has Coral", hasCoral());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
