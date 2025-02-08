// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorDefaultCommand extends Command {
  private final ElevatorSubsystem m_subsystem;
  private XboxController m_opperatorController;

  /**
   * Creates a new ElevatorDefaultCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorDefaultCommand(ElevatorSubsystem subsystem, XboxController operatorController)  {
    m_subsystem = subsystem;
    m_opperatorController = operatorController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Note: We negate the axis values so that pushing the joystick forwards
    // results in upward motion
    m_subsystem.setMotorSpeed(-m_opperatorController.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
