// This is the default auto command that drives the robot across the auto line.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousDrive extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_leftSpeed = 0.0;
  private double m_rightSpeed = 0.0;

  /**
   * Creates a new AutonomousDrive2Sec.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDrive(DriveSubsystem subsystem, double leftSpeed, double rightSpeed) 
  {
    m_subsystem = subsystem;
    
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setTankSpeeds(0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setTankSpeeds(m_leftSpeed, m_rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTankSpeeds(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // <-- VERY SKETCHY !!
  }
}
