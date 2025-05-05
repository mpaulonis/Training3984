package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {

  private SparkMax spark;
  private SparkMax sparkFollower;

  private SparkMaxConfig config;
  private SparkMaxConfig configFollower;

  private RelativeEncoder internalEncoder;

  private final double encoderPositionStartRev =
      ElevatorConstants.elevatorGearReduction
          * ElevatorConstants.elevatorMinHeightInch
          / (ElevatorConstants.sprocketPitchDiameter * Math.PI);

  /** Construct an elevator */
  public ElevatorIOSparkMax() {
    spark = new SparkMax(ElevatorConstants.leaderCanId, MotorType.kBrushless);
    sparkFollower = new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless);
    config = new SparkMaxConfig();
    configFollower = new SparkMaxConfig();
    internalEncoder = spark.getEncoder();

    config
        .inverted(ElevatorConstants.leaderIsInverted)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit);

    configFollower.follow(ElevatorConstants.leaderCanId, ElevatorConstants.invertFollower);

    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkFollower.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    internalEncoder.setPosition(encoderPositionStartRev);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // native NEO encoder position is in revolutions
    inputs.positionInch =
        internalEncoder.getPosition()
            / ElevatorConstants.elevatorGearReduction
            * ElevatorConstants.sprocketPitchDiameter
            * Math.PI;
    // native NEO encoder velocity is in revolutions per minute
    inputs.velocityInchPerSec =
        internalEncoder.getVelocity()
            / ElevatorConstants.elevatorGearReduction
            / 60.0
            * ElevatorConstants.sprocketPitchDiameter
            * Math.PI;
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
