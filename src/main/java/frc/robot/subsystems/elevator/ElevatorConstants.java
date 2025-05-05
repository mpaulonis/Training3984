package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int leaderCanId;
  public static final int followerCanId;
  public static final boolean leaderIsInverted;
  // is the follower inverted relative to the leader?
  public static final boolean invertFollower;

  public static final double sprocketPitchDiameter;
  public static final double elevatorGearReduction;

  public static final double elevatorMinHeightInch;
  public static final double elevatorMaxHeightInch;
  public static final double elevatorMidHeightInch;

  // how close to the PID goal to be considered at goal?
  public static final double elevatorGoalToleranceInch;
  // how close to the minimum height to be considered down?
  public static final double elevatorIsDownToleranceInch;

  public static final double elevatorMaxVelocity;
  public static final double elevatorMaxAcceleration;

  public static final int elevatorCurrentLimit;

  static {
    leaderCanId = 20;
    followerCanId = 21;
    leaderIsInverted = true;
    invertFollower = true;

    sprocketPitchDiameter = 1.7567;
    elevatorGearReduction = 15.0;

    elevatorMaxHeightInch = 28.0;
    elevatorMinHeightInch = 0.0;
    elevatorMidHeightInch = 14.0;

    elevatorIsDownToleranceInch = 1.0;
    elevatorGoalToleranceInch = 0.2;

    elevatorMaxVelocity = 30.0;
    elevatorMaxAcceleration = 60.0;

    elevatorCurrentLimit = 40;
  }
}
