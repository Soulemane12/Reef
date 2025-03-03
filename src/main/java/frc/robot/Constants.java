// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ElevatorConstants{
    public static final int kElevatorMotorID1 = 20;
    public static final int kElevatorMotorID2 = 21;

    // Elevator positions
    public static final double kElevatorL0Position = 0.25;  // Point 0 position
    public static final double kElevatorL2Position = 1.0;   // L2 position
    public static final double kElevatorL3Position = 3.0;   // L3 position
    public static final double kElevatorL4Position = 4.5;   // L4 position
  }
  public static class ShooterConstants{
    public static final int kShooterMotorID = 30;

    
    
  }
  public static class PivotConstants {
    public static final int kPivotMotorID = 30;  // Same as ShooterMotorID

    // Pivot positions in radians
    public static final double kPivotL0Position = 0.75;     // L0 position
    public static final double kPivotL2Position = -1.83;    // L2 position
    public static final double kPivotL3Position = -1.4;     // L3 position
    public static final double kPivotL4Position = -1.5;     // L4 position
    public static final double kPivotInPosition = -2.94;    // In position
    public static final double kPivotParallelPosition = -3.0;  // Parallel to ground
  }
}
