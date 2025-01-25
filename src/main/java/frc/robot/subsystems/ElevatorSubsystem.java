// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private double m_motorSpeed = 0;

  private final SparkMax m_motor = new SparkMax(ElevatorConstants.kMotorCANID, 
                                                            MotorType.kBrushless);

  private final DigitalInput m_Sensor = new DigitalInput(ElevatorConstants.kSensorChannel);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  public void setMotorSpeed(double speed)
  {
    m_motor.set(speed);
    m_motorSpeed = speed;
  }
  
  // Returns the last COMMANDED speed
  public double getMotorSpeed()
  {
    return m_motorSpeed;
  }

  public boolean isElevatorDown()
  {
    boolean sensorRead = m_Sensor.get();
    return ElevatorConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    RelativeEncoder motorEncoder = m_motor.getEncoder();

    if (isElevatorDown() == true) {
      motorEncoder.setPosition(0.0);
    }

    SmartDashboard.putNumber("Elevator Encoder", motorEncoder.getPosition());
    SmartDashboard.putBoolean("Is Elevator Down", isElevatorDown());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
