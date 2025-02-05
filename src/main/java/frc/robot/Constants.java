// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //
 // When updating this file, make sure that the following rules apply:
 //
 // 1. CAN IDs must be unique across the whole robot (assuming we only have a single
 // CAN bus which is true this year).
 // 2. Sensor channels must be unique across the whole robot (assuming we're using DIO
 // channels for light break or switch-based sensors).
 // 3. Pneumatics channels must be unique across the whole robot (assuming we're using
 // a single pneumatics hub)

 // Current CAN IDs:
 //   01 - Pneumatics hub
 //   10 - Drive left primary
 //   11 - Drive left follower
 //   20 - Drive right primary
 //   21 - Drive right follower
 //   30 - LaserCAN
 //   40 - Coral handler input side motor  
 //   41 - Coral handler output side motor
 //   50 - Algae handler motor
 //   60 - Elevator motor
 //   70 - Coral ramp intake motor
 //
 // Current DIO Channels:
 //
 // 0 - Coral handler front sensor
 // 1 - Coral handler rear sensor
 // 2 - Coral ramp intake sensor
 // 3 - Algae sensor
 // 4 - Elevator bottom sensor
 // 5 -
 // 6 -
 // 7 -
 //

 public final class Constants {
  public static class RobotConstants {
    public static final double minPnuematicsPressure = 80.0;
    public static final double maxPnuematicsPressure = 120.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int kZeroElevator = 4; // Button Y
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID = 10;
    public static final int kOptionalLeftMotorCANID = 11;

    public static final int kRightMotorCANID = 20;
    public static final int kOptionalRightMotorCANID = 21;

    public static final double kTurnDivider = 1.0;
    public static final double kSpeedDivider = 1.0;

    public static final int kLaserCanCANID = 30;

    // SmartDashboard update frequency for drive subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;
  }

  public static class AutoConstants {
    public static final double kPassLineSpeed = 0.40;
    public static final double kTurnInnerSpeed = 0.35;
  }

  public static class IntakeConstants {
    public static final int kMotorCANID = 70;

    public static final int kSensorChannel = 2;
    public static final boolean kSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for intake subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 22;
  }

  public static class CoralConstants {
    public static final int kIntakeMotorCANID = 40;
    public static final int kOutputMotorCANID = 41;

    // Light break sensor indicating that a coral is visible at the
    // front (output) side of the coral handler.
    public static final int kFrontSensorChannel = 0;
    public static final boolean kFrontSensorFalseIsEmpty = false;

    // Light break sensor indicating that a coral is visible at the
    // back (input) side of the coral handler.
    public static final int kBackSensorChannel = 1;
    public static final boolean kBackSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for coral subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 10;
  }

  public static class AlgaeConstants {
    public static final int kMotorCANID = 50;

    public static final int kPneumaticsHubCANid = 1;

    public static final int kSensorChannel = 3;
    public static final boolean kSensorFalseIsEmpty = false;

    // Temp Values, Needs to be changed when solunoid is set up onto the robot
    public static final int kExtendChannel = 8;
    public static final int kRetractChannel = 7;

    public static final DoubleSolenoid.Value kOffState = DoubleSolenoid.Value.kOff;
    public static final DoubleSolenoid.Value kDeployedState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value kReteactedState = DoubleSolenoid.Value.kReverse;

    // SmartDashboard update frequency for algae subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 25;
  }

  public static class ElevatorConstants {
    public static final int kMotorCANID = 60;
    public static final double kMotorZeroSpeed = 0.1;

    public static final int kSensorChannel = 4;
    public static final boolean kSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for elevator subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;
  }
}
