package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double internalVelocityInchPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double positionInch = 0.0;
    public double velocityInchPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the arm1 motor(s) at the specified voltages. */
  public default void setVoltage(double volts) {}

  /** Stop in closed loop. */
  public default void stop() {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean arm1Brake) {}
}
