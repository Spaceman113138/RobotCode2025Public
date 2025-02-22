// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.chariot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.gslam.GenericSlamElevator;
import org.littletonrobotics.frc2025.util.gslam.GenericSlamElevator.SlamElevatorGoal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Chariot extends GenericSlamElevator<Chariot.Goal> {
  static final double drumRadius = Units.inchesToMeters(0.5);

  public static final LoggedTunableNumber occupiedVolts =
      new LoggedTunableNumber("Chariot/OccupiedVolts", 5.0);
  public static final LoggedTunableNumber floorIntakeVolts =
      new LoggedTunableNumber("Chariot/FloorIntakeVolts", 8.0);
  public static final LoggedTunableNumber halfOutPositionInches =
      new LoggedTunableNumber("Chariot/StupidDeployPositionInches", 5.0);

  @Getter
  @RequiredArgsConstructor
  public enum Goal implements SlamElevatorGoal {
    IDLE(() -> 0.0, true, SlamElevatorState.IDLING),
    RETRACT(
        new LoggedTunableNumber("Chariot/RetractCurrent", -30.0),
        true,
        SlamElevatorState.RETRACTING),
    DEPLOY(
        new LoggedTunableNumber("Chariot/DeployCurrent", 30.0), true, SlamElevatorState.EXTENDING),
    HALF_OUT(
        new LoggedTunableNumber("Chariot/StupidDeployCurrent", 10.0),
        true,
        SlamElevatorState.EXTENDING);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private RollerSystemIO rollerIO;
  private final RollerSystemIOInputsAutoLogged rollerInputs = new RollerSystemIOInputsAutoLogged();

  @Setter @Getter private Goal goal = Goal.IDLE;
  @Setter private double intakeVolts = 0.0;
  @AutoLogOutput private double home = 0.0;
  @AutoLogOutput @Getter private double position = 0.0;

  @Setter private BooleanSupplier coastOverride = () -> false;

  public Chariot(ChariotIO chariotIO, RollerSystemIO rollerIO) {
    super("Chariot/Chariot", chariotIO, 0.2, 0.5);
    this.rollerIO = rollerIO;

    new Trigger(() -> getGoal() == Goal.RETRACT && slammed)
        .onTrue(Commands.runOnce(() -> home = inputs.positionRads * drumRadius));
    new Trigger(() -> getGoal() == Goal.DEPLOY && slammed)
        .onTrue(
            Commands.runOnce(
                () ->
                    home =
                        inputs.positionRads * drumRadius
                            - SuperstructureConstants.chariotMaxExtension));
  }

  public void periodic() {
    super.periodic();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Chariot/Roller", rollerInputs);

    if (Constants.getRobot() == Constants.RobotType.DEVBOT) {
      slammed = true;
    }

    position = inputs.positionRads * drumRadius - home;
    if (getGoal() == Goal.HALF_OUT
        && position >= Units.inchesToMeters(halfOutPositionInches.get())) {
      slammed = true;
    }

    rollerIO.runVolts(intakeVolts);
  }
}
