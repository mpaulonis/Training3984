package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionInch = 0.0;
    public double velocityInchPerSec = 0.0;
    public double tempCelsius = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    // if you had other sensors on the elevator such as an external encoder
    // for height or limit switches for full down / full up, you would
    // need to include these here
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the elevator motor(s) at the specified voltages. */
  public default void setVoltage(double volts) {}

  /** Stop the motors. */
  public default void stop() {}

  // If we were running PID control on the controller hardware, we would
  // need to add a method to send the PID setpoint to the controller

  // if we wanted to do anything else to the control hardware while the
  // robot is running such as changing between brake and coast or
  // changing PID tuning, you would need to include these methods here
}
