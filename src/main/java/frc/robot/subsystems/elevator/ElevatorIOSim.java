package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private double LOOP_PERIOD_SEC = 0.02;

  private ElevatorSim elevSim;
  private double appliedVolts = 0.0;

  public ElevatorIOSim() {

    elevSim =
        new ElevatorSim(
            DCMotor.getNEO(2),
            ElevatorConstants.elevatorGearReduction,
            9.072,
            Units.inchesToMeters(ElevatorConstants.sprocketPitchDiameter / 2),
            Units.inchesToMeters(ElevatorConstants.elevatorMinHeightInch),
            Units.inchesToMeters(ElevatorConstants.elevatorMaxHeightInch),
            true,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Reset voltage when disabled
    if (DriverStation.isDisabled()) {
      setVoltage(0.0);
    }

    // Update sim state
    elevSim.update(LOOP_PERIOD_SEC);

    // Log sim data
    inputs.positionInch = Units.metersToInches(elevSim.getPositionMeters());
    inputs.velocityInchPerSec = Units.metersToInches(elevSim.getVelocityMetersPerSecond());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = elevSim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    elevSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
