// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants.ReefHeight;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final double reversedHeightOffset = Units.inchesToMeters(5.0);
  private static final double algaeIntakeAngle = -105.0;
  private static final double groundToCarriageZero = carriageOrigin3d.getZ();

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOW(
        new LoggedTunableNumber("Superstructure/Stow/Elevator", 0.0),
        new LoggedTunableNumber("Superstructure/Stow/Pivot", 0.0)),
    L1(
        new LoggedTunableNumber("Superstructure/L1/Elevator", 0.1),
        new LoggedTunableNumber("Superstructure/L1/Pivot", 40.0)),
    L2(
        new LoggedTunableNumber(
            "Superstructure/L2/Elevator",
            ReefHeight.L2.height
                - groundToCarriageZero
                - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()),
        new LoggedTunableNumber("Superstructure/L2/Pivot", -45.0)),
    L3(
        new LoggedTunableNumber(
            "Superstructure/L3/Elevator",
            ReefHeight.L3.height
                - groundToCarriageZero
                - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()),
        new LoggedTunableNumber("Superstructure/L3/Pivot", -45.0)),
    L4(
        new LoggedTunableNumber(
            "Superstructure/L4/Elevator",
            ReefHeight.L4.height
                - groundToCarriageZero
                - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()),
        new LoggedTunableNumber("Superstructure/L4/Pivot", -45.0)),
    ALGAE_FLOOR_INTAKE(
        "AlgaeFloorIntake",
        Units.inchesToMeters(16.5)
            - groundToCarriageZero
            - (pivotToGripper / 2.0 * Rotation2d.fromDegrees(algaeIntakeAngle).getSin()),
        algaeIntakeAngle),
    ALGAE_L2_INTAKE(
        "AlgaeL2Intake",
        ((ReefHeight.L2.height + ReefHeight.L3.height) / 2.0 - groundToCarriageZero)
            - (pivotToGripper / 2.0 * Rotation2d.fromDegrees(algaeIntakeAngle).getSin()),
        algaeIntakeAngle),
    ALGAE_L3_INTAKE(
        "AlgaeL2Intake",
        (ReefHeight.L3.height + Units.inchesToMeters(5.5) - groundToCarriageZero)
            - (pivotToGripper / 2.0 * Rotation2d.fromDegrees(algaeIntakeAngle).getSin()),
        algaeIntakeAngle),
    THROW(() -> elevatorHeightMeters, () -> pivotSafeAngle.getDegrees()),
    PRE_PROCESSOR("Processing", 0.05, -80.0),
    ALGAE_STOW("AlgaeStow", 0.0, 25.0),
    ALGAE_STOW_FRONT("AlgaeStowFront", 0.1, pivotSafeAngle.getDegrees()),
    L3_CORAL_REVERSED(
        "L3Reversed",
        ReefHeight.L3.height
            - groundToCarriageZero
            - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()
            + reversedHeightOffset,
        -225.0),
    L4_CORAL_REVERSED(
        "L4Reversed",
        ReefHeight.L4.height
            - groundToCarriageZero
            - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()
            + reversedHeightOffset,
        -225.0),
    L3_CORAL_UNREVERSED(
        "L3Unreversed",
        ReefHeight.L3.height
            - groundToCarriageZero
            - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()
            + reversedHeightOffset,
        -110.0),
    L4_CORAL_UNREVERSED(
        "L4Unreversed",
        ReefHeight.L4.height
            - groundToCarriageZero
            - pivotLength * Rotation2d.fromDegrees(-45.0).getSin()
            + reversedHeightOffset,
        -110.0);

    private final SuperstructurePose pose;

    private Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromDegrees(pivotAngle.getAsDouble())));
    }

    private Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }
  }

  SuperstructurePose() {
    this(() -> 0.0, Rotation2d::new);
  }
}
