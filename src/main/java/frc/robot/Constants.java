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
public final class Constants {
  public static class RobotConstants {
    public static final double minPnuematicsPressure = 80.0;
    public static final double maxPnuematicsPressure = 120.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID = 10;
    public static final int kOptionalLeftMotorCANID = 11;

    public static final int kRightMotorCANID = 20;
    public static final int kOptionalRightMotorCANID = 21;

    public static final double kTurnDivider = 1.0;
    public static final double kSpeedDivider = 1.0;

    public static final int kLaserCanCANID = 30;
  }

  public static class AutoConstants {
    public static final double kPassLineSpeed = 0.40;
    public static final double kTurnInnerSpeed = 0.35;
  }

  public static class CoralConstants {
    public static final int kIntakeMotorCANID = 40;
    public static final int kOutputMotorCANID = 41;

    public static final int kSensorChannel = 0;
    public static final boolean kSensorFalseIsEmpty = false;
  }

  public static class AlgaeConstants {
    public static final int kMotorCANID = 50;

    public static final int kPneumaticsHubCANid = 1;

    public static final int kSensorChannel = 1;
    public static final boolean kSensorFalseIsEmpty = false;

    // Temp Values, Needs to be changed when solunoid is set up onto the robot
    public static final int kExtendChannel = 8;
    public static final int kRetractChannel = 7;

    public static final DoubleSolenoid.Value kOffState = DoubleSolenoid.Value.kOff;
    public static final DoubleSolenoid.Value kDeployedState = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value kReteactedState = DoubleSolenoid.Value.kReverse;
  }
}
