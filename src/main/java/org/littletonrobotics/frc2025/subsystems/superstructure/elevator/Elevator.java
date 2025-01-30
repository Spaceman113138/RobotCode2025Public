// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public static final double drumRadiusMeters = Units.inchesToMeters(6.0);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 5000);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 2000);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 5);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 50);
  private static final LoggedTunableNumber maxTorque =
      new LoggedTunableNumber("Elevator/MaxTorqueNm", 20.0);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -3.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.5);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingCurrentLimitAmps", 0.05);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.1);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = DriverStation::isDisabled;

  @AutoLogOutput(key = "Elevator/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  private ExponentialProfile profile;
  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;

  @AutoLogOutput(key = "Elevator/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput(key = "Elevator/Homed")
  @Getter
  private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    profile = new ExponentialProfile(fromMaxTorque(maxTorque.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxTorque.hasChanged(hashCode())) {
      profile = new ExponentialProfile(fromMaxTorque(maxTorque.get()));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    final boolean shouldRunProfile =
        !stopProfile && !coastOverride.getAsBoolean() && !disabledOverride.getAsBoolean() && homed;
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);
    // Run profile
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(
                  goal.get().position, 0.0, SuperstructureConstants.elevatorHeightMeters),
              goal.get().velocity);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      io.runPosition(
          setpoint.position / drumRadiusMeters,
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * SuperstructureConstants.elevatorAngle.getSin());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec", inputs.velocityRadPerSec * drumRadiusMeters);
  }

  public void setGoal(DoubleSupplier goal) {
    setGoal(() -> new State(goal.getAsDouble(), 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    atGoal = false;
    this.goal = goal;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride =
        () -> this.disabledOverride.getAsBoolean() && disabledOverride.getAsBoolean();
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> inputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .finallyDo(
            () -> {
              stopProfile = false;
              homedPosition = inputs.positionRad;
              homed = true;
            });
  }

  /** Get position of elevator in meters with 0 at home */
  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.positionRad - homedPosition) * drumRadiusMeters;
  }

  public double getGoalMeters() {
    return goal.get().position;
  }

  @AutoLogOutput(key = "Elevator/AvgCurrent")
  private double getCurrent() {
    return Arrays.stream(inputs.currentAmps).average().getAsDouble();
  }

  private static Constraints fromMaxTorque(double maxTorque) {
    return Constraints.fromStateSpace(
        maxTorque / ElevatorIOSim.gearbox.KtNMPerAmp,
        ElevatorIOSim.A.get(1, 1),
        ElevatorIOSim.B.get(1));
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
