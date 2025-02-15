// This is the default auto command that drives the robot across the auto line.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousDrivePID extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_leftTarget = 0.0;
  private double m_rightTarget = 0.0;
  private double m_tolerance = 0.0;

  /**
   * Creates a new AutonomousDrivePID.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDrivePID(DriveSubsystem subsystem, double leftTarget, double rightTarget, double tolerance) 
  {
    m_subsystem = subsystem;
    
    m_leftTarget = leftTarget;
    m_rightTarget = rightTarget;

    m_tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLeftTargetPosition(m_leftTarget);
    m_subsystem.setRightTargetPosition(m_rightTarget);
    System.out.println("ADPID Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This will stop the differential drive saftey warning
    m_subsystem.callDrivetrainFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto PID Ended");
    m_subsystem.setLeftTargetPosition(0);
    m_subsystem.setRightTargetPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isAtTarget(m_tolerance);
  }
}
