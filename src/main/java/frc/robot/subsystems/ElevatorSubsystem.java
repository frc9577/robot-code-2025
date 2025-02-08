// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment. What does this class do?

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

// TODO: This class likely needs to be significantly reworked since we need to be
// able to control the position of the elevator rather than the speed. We'll likely
// use the internal SparkMax position controller to do this so your methods will 
// end up being along the lines of setTargetHeight(), getTargetHeight(), getCurrentHeight(),
// etc.

public class ElevatorSubsystem extends SubsystemBase {
  private double m_motorSpeed = 0;

  private final TalonFX m_motor = new TalonFX(ElevatorConstants.kMotorCANID);

  private final DigitalInput m_LineBreakSensor = new DigitalInput(ElevatorConstants.kSensorChannel);
  private boolean m_sensorBroken = false;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // We need to know if the motor controller we need is
    // actually present on the CAN bus and, unfortunately, the 
    // constructor doesn't seem to throw an exception in this case. 
    // Let's query for firmware error status and use this for now.

    // TODO: Investigate a version of this for talonFX
    //if (m_motor.getFaults().firmware)
    //{
    //  throw new RuntimeException("Elevator subsystem motor not present");
    //}
  }

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
    boolean sensorRead = m_LineBreakSensor.get();
    return ElevatorConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  // Returns the raw encoder position.
  public double getPosition()
  {
    // TODO: Return the current position of the elevator (in metres above 0?)
    return m_motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean elevatorDown = isElevatorDown();

    if ((elevatorDown == true) && (m_sensorBroken == false)) {
      m_motor.setPosition(0);
      m_sensorBroken = true;
    }

    if ((elevatorDown == false) && (m_sensorBroken == true)) {
      m_sensorBroken = false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
