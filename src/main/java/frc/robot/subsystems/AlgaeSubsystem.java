// TODO: Class header comment.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final DoubleSolenoid m_Solenoid = new DoubleSolenoid(
                  AlgaeConstants.kPneumaticsHubCANid, PneumaticsModuleType.REVPH, 
                  AlgaeConstants.kExtendChannel, AlgaeConstants.kRetractChannel);

    private final DigitalInput m_Sensor = new DigitalInput(AlgaeConstants.kSensorChannel);

    private double m_MotorSpeed = 0;
    private final SparkMax m_Motor = new SparkMax(AlgaeConstants.kMotorCANID, MotorType.kBrushless);

    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() 
    {
        // We need to know if the motor controller we need is
        // actually present on the CAN bus and, unfortunately, the 
        // constructor doesn't seem to throw an exception in this case. 
        // Let's query for CAN error status and use this for now.
        if (m_Motor.getFaults().can)
        {
        throw new RuntimeException("Algae subsystem motor not present");
        }
    }

    public enum State {
      OFF,
      DEPLOYED,
      RETRACTED
    }

    private State m_currentState = State.OFF;

    // Sets the position (state) of the mechanism.
    public void setPosition(State newState)
    {
        m_currentState = newState;
        DoubleSolenoid.Value newValue = DoubleSolenoid.Value.kOff;

        switch (m_currentState) 
        {
        case OFF:
            newValue = AlgaeConstants.kOffState;
            break;

        case DEPLOYED:
            newValue = AlgaeConstants.kDeployedState;
            break;

        case RETRACTED:
            newValue = AlgaeConstants.kReteactedState;
            break;
        }

        m_Solenoid.set(newValue);
    }

    // gets the last commanded position (state) of the mechanism
    public State getPosition()
    {
        return m_currentState;
    }

    // checks if you have an algae in posession
    public boolean hasAlgae()
    {
        boolean sensorRead = m_Sensor.get();
        return AlgaeConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
    }

    // sets the speed of the intake
    public void setIntakeSpeed(double speed) {
        m_Motor.set(speed);
        m_MotorSpeed = speed;
    }

    // gets the last commanded speed for the intake
    public double getIntakeSpeed() {
        return m_MotorSpeed;
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
