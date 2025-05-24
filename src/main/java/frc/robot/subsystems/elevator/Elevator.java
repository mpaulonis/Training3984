package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorFeedforward ffModel;
  private final ProfiledPIDController pid;
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private boolean elevatorClosedLoop = false;
  private double pidOutput;
  private double feedforwardOutput;
  private double goalInches = ElevatorConstants.elevatorMinHeightInch;
  private String elevatorStatus;

  // Create a Mechanism2d display of the elevator
  // To display in AdvantageScope, lengths must be in meters
  private final Mechanism2d mech2d =
      new Mechanism2d(Units.inchesToMeters(31.5), Units.inchesToMeters(80));
  private final MechanismRoot2d baseRoot =
      mech2d.getRoot("BaseRoot", Units.inchesToMeters(21), Units.inchesToMeters(4));
  private final MechanismLigament2d base =
      baseRoot.append(new MechanismLigament2d("Base", Units.inchesToMeters(37), 90));
  private final MechanismRoot2d firstRoot =
      mech2d.getRoot("FirstRoot", Units.inchesToMeters(21), Units.inchesToMeters(4));
  private final MechanismLigament2d first =
      firstRoot.append(new MechanismLigament2d("First", Units.inchesToMeters(38), 90));
  private final MechanismRoot2d secondRoot =
      mech2d.getRoot("SecondRoot", Units.inchesToMeters(21), Units.inchesToMeters(13));
  private final MechanismLigament2d secondDown =
      secondRoot.append(new MechanismLigament2d("SecondDown", Units.inchesToMeters(7), -90));
  private final MechanismLigament2d secondUp =
      secondRoot.append(new MechanismLigament2d("SecondUp", Units.inchesToMeters(1), 90));
  private final MechanismLigament2d secondMount =
      secondRoot.append(new MechanismLigament2d("SecondMount", Units.inchesToMeters(7), 0));

  /**
   * Creates a new Elevator.
   *
   * @param io The IO object to interface with the elevator, whether real hardware IO or simulation
   *     IO
   */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Put Mechanism2d to SmartDashboard
    base.setColor(new Color8Bit(Color.kBlue));
    first.setColor(new Color8Bit(Color.kGreen));
    secondDown.setColor(new Color8Bit(Color.kYellow));
    secondUp.setColor(new Color8Bit(Color.kYellow));
    secondMount.setColor(new Color8Bit(Color.kYellow));
    SmartDashboard.putData("Elevator Sim", mech2d);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new ElevatorFeedforward(0.0, 0.17, 0.40, 0.02);
        pid =
            new ProfiledPIDController(
                2.5,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.elevatorMaxVelocity,
                    ElevatorConstants.elevatorMaxAcceleration));

        break;
      case REPLAY:
        ffModel = new ElevatorFeedforward(0.0, 0.17, 0.40, 0.02);
        pid =
            new ProfiledPIDController(
                2.5,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.elevatorMaxVelocity,
                    ElevatorConstants.elevatorMaxAcceleration));

        break;
      case SIM:
        ffModel = new ElevatorFeedforward(0.0, 0.17, 0.40, 0.02);
        pid =
            new ProfiledPIDController(
                5,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.elevatorMaxVelocity,
                    ElevatorConstants.elevatorMaxAcceleration));

        break;
      default:
        ffModel = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
        pid = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        break;
    }
    pid.setTolerance(ElevatorConstants.elevatorGoalToleranceInch);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (elevatorClosedLoop) {
      pidOutput = pid.calculate(inputs.positionInch, goalInches);
      feedforwardOutput = ffModel.calculate(pid.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);

      Logger.recordOutput("Elevator/atGoal", pid.atGoal());
      Logger.recordOutput("Elevator/heightGoalInch", goalInches);
      Logger.recordOutput("Elevator/heightSPInch", pid.getSetpoint().position);
      Logger.recordOutput("Elevator/heightSPInchPerSec", pid.getSetpoint().velocity);
      Logger.recordOutput("Elevator/feedbackOP", pidOutput);
      Logger.recordOutput("Elevator/feedforwardOP", feedforwardOutput);

    } else {
      pid.reset(inputs.positionInch);
    }

    Logger.recordOutput("Elevator/heightInternalInch", inputs.positionInch);
    Logger.recordOutput("Elevator/motorVolts", inputs.appliedVolts);
    Logger.recordOutput("Elevator/elevatorIsAtTarget", atGoal());
    Logger.recordOutput("Elevator/status", elevatorStatus);

    // Update the Mechanism2d visualization positions
    firstRoot.setPosition(Units.inchesToMeters(21), Units.inchesToMeters(4 + inputs.positionInch));
    secondRoot.setPosition(
        Units.inchesToMeters(21), Units.inchesToMeters(13 + (2 * inputs.positionInch)));
  }

  /** Stop the elevator */
  public void stop() {
    io.stop();
    elevatorClosedLoop = false;
  }

  /** Returns the current elevator height in inches. */
  @AutoLogOutput(key = "Elevator/heightInternalInch")
  public double getHeightInch() {
    return inputs.positionInch;
  }

  /** Set the goal for the elevator in inches. Put controller in auto if not already. */
  public void setGoalInch(double setpointInch) {
    elevatorClosedLoop = true;
    goalInches =
        MathUtil.clamp(
            setpointInch,
            ElevatorConstants.elevatorMinHeightInch,
            ElevatorConstants.elevatorMaxHeightInch);
  }

  /**
   * Check if the elevator has reached the closed-loop height goal. The tolerance is set with
   * ElevatorConstants.elevatorGoalToleranceInch.
   *
   * @return True if elevator height is within tolerance of the goal
   */
  public boolean atGoal() {
    return pid.atGoal();
  }
  /**
   * Check if elevator is within a tolerance of the fully down position. Typically used as a
   * permissive in commands.
   *
   * @return True if elevator is considered fully down
   */
  @AutoLogOutput
  public boolean isDown() {
    return inputs.positionInch
        <= ElevatorConstants.elevatorMinHeightInch + ElevatorConstants.elevatorIsDownToleranceInch;
  }

  /**
   * Returns a command to stop the elevator motors. In this state the elevator can move by the force
   * of gravity, but if the motors are in brake mode, this movement will be slow.
   */
  public Command stopCommand() {
    return new InstantCommand(() -> stop(), this);
  }

  /**
   * Returns a command to set the elevator controller goal to the current elevator position.
   *
   * @param hardHold If true, resets the trapezoid profile velocity to zero to force a "hard" hold.
   *     If false, the acceleration limit will be respected so the arm position will overshoot and
   *     then recover to the position when the command was iniitiated.
   */
  public Command holdCommand(boolean hardHold) {
    return new InstantCommand(
        () -> {
          if (hardHold) pid.reset(new TrapezoidProfile.State(inputs.positionInch, 0));
          setGoalInch(inputs.positionInch);
        },
        this);
  }

  /** Returns a command to set the arm controller goal to the supplied value. */
  public Command toTargetCommand(double targetInch) {
    return new InstantCommand(() -> setGoalInch(targetInch), this);
  }
}
