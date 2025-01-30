// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Dispenser {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-140.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(160.0);
  private static final double maxAngleRad = calculateFinalAngle(maxAngle).getRadians();
  private static final double minAngleRad = calculateFinalAngle(minAngle).getRadians();

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Dispenser/kP", 4000.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Dispenser/kD", 2000.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Dispenser/kS", 1.2);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Dispenser/kG", 0.0);
  private static final LoggedTunableNumber maxTorque =
      new LoggedTunableNumber("Dispenser/MaxTorqueNm", 2.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber algaeIntakeCurrentThresh =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeCurrentThreshold", 40.0);
  public static final LoggedTunableNumber gripperIntakeCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeCurrent", 30.0);
  public static final LoggedTunableNumber gripperDispenseCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeDispenseCurrent", -30.0);
  public static final LoggedTunableNumber tunnelDispenseVolts =
      new LoggedTunableNumber("Dispenser/TunnelDispenseVolts", 6.0);
  public static final LoggedTunableNumber tunnelIntakeVolts =
      new LoggedTunableNumber("Dispenser/TunnelIntakeVolts", -6.0);

  // Hardware
  private final DispenserIO pivotIO;
  private final DispenserIOInputsAutoLogged pivotInputs = new DispenserIOInputsAutoLogged();
  private final RollerSystemIO tunnelIO;
  private final RollerSystemIOInputsAutoLogged tunnelInputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = DriverStation::isDisabled;

  @AutoLogOutput(key = "Dispenser/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  // Control
  @Getter
  @AutoLogOutput(key = "Dispenser/MeasuredAngle")
  private Rotation2d finalAngle = new Rotation2d();

  private ExponentialProfile profile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;

  @Getter
  @AutoLogOutput(key = "Dispenser/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private double tunnelVolts = 0.0;
  @Setter private double gripperCurrent = 0.0;

  @AutoLogOutput(key = "Dispenser/HasAlgae")
  private boolean hasAlgae = false;

  private Debouncer algaeDebouncer = new Debouncer(0.1);

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Davis Dispenser pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Davis Dispenser encoder disconnected!", Alert.AlertType.kWarning);
  private final Alert tunnelDisconnectedAlert =
      new Alert("Davis Dispenser tunnel disconnected!", Alert.AlertType.kWarning);
  private final Alert gripperDisconnectedAlert =
      new Alert("Davis Dispenser gripper disconnected!", Alert.AlertType.kWarning);

  public Dispenser(DispenserIO pivotIO, RollerSystemIO tunnelIO, RollerSystemIO gripperIO) {
    this.pivotIO = pivotIO;
    this.tunnelIO = tunnelIO;
    this.gripperIO = gripperIO;

    profile = new ExponentialProfile(fromMaxTorque(maxTorque.get()));

    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      new Trigger(() -> DriverStation.getStickButtonPressed(2, 1))
          .onTrue(Commands.runOnce(() -> hasAlgae = !hasAlgae));
    }
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Dispenser/Pivot", pivotInputs);
    tunnelIO.updateInputs(tunnelInputs);
    Logger.processInputs("Dispenser/Tunnel", tunnelInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Dispenser/Gripper", gripperInputs);

    pivotMotorDisconnectedAlert.set(!pivotInputs.motorConnected);
    pivotEncoderDisconnectedAlert.set((!pivotInputs.encoderConnected));
    tunnelDisconnectedAlert.set(!tunnelInputs.connected);
    gripperDisconnectedAlert.set(!gripperInputs.connected);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivotIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxTorque.hasChanged(hashCode())) {
      profile = new ExponentialProfile(fromMaxTorque(maxTorque.get()));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Calculate combined angle
    finalAngle = calculateFinalAngle(pivotInputs.internalPosition);

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile && !coastOverride.getAsBoolean() && !disabledOverride.getAsBoolean();
    Logger.recordOutput("Dispenser/RunningProfile", shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState = new State(MathUtil.clamp(goal.getAsDouble(), minAngleRad, maxAngleRad), 0.0);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(setpoint.position).plus(SuperstructureConstants.elevatorAngle),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * finalAngle.getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log state
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", setpoint.position);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", goalState.position);
    } else {
      // Reset setpoint
      setpoint = new State(finalAngle.getRadians(), 0.0);
      // Clear logs
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", 0.0);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", 0.0);
    }

    // Run tunnel and gripper
    tunnelIO.runVolts(tunnelVolts);
    gripperIO.runTorqueCurrent(gripperCurrent);

    // Check algae
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      hasAlgae =
          algaeDebouncer.calculate(
              Math.abs(gripperInputs.supplyCurrentAmps) >= algaeIntakeCurrentThresh.get());
    }

    // Log state
    Logger.recordOutput("Dispenser/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Dispenser/DisabledOverride", disabledOverride.getAsBoolean());
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    setGoal(() -> goal.get().getRadians());
  }

  public void setGoal(DoubleSupplier goal) {
    atGoal = false;
    this.goal = goal;
  }

  public double getGoal() {
    return goal.getAsDouble();
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride =
        () -> this.disabledOverride.getAsBoolean() && disabledOverride.getAsBoolean();
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    pivotIO.setBrakeMode(enabled);
  }

  private static ExponentialProfile.Constraints fromMaxTorque(double maxTorque) {
    return ExponentialProfile.Constraints.fromStateSpace(
        maxTorque / DispenserIOSim.gearbox.KtNMPerAmp,
        DispenserIOSim.A.get(1, 1),
        DispenserIOSim.B.get(1));
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
              pivotIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Dispenser/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> pivotInputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Dispenser/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public static Rotation2d calculateFinalAngle(Rotation2d pivotAngle) {
    return new Rotation2d(
        pivotAngle.getRadians() - SuperstructureConstants.elevatorAngle.getRadians());
  }
}
