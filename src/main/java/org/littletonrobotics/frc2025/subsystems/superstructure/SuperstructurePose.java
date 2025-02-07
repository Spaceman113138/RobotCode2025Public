// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants.ReefHeight;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final double algaeIntakeAngle = -105.0;
  private static final double groundToCarriageZero = dispenserOrigin2d.getY();
  private static final double tunnelEjectMeters = Units.inchesToMeters(12.0);
  private static final double tunnelEjectMetersReverse = Units.inchesToMeters(10.0);

  private static final double L2Angle = ReefHeight.L2.pitch;
  private static final double L3Angle = ReefHeight.L3.pitch;
  private static final double L4Angle = -40.0;
  private static final double L3AngleAlgae = L3Angle - 180.0;
  private static final double L4AngleAlgae = L4Angle + 10.0 - 180.0;

  @RequiredArgsConstructor
  @Getter
  /**
   * Dispenser pose relative to branch on ground (looking at the robot from the left side) Rotation
   * is just the rotation of dispenser when scoring.
   */
  public enum DispenserPose {
    L1(
        new Pose2d(
            Units.inchesToMeters(15.0), dispenserOrigin2d.getY(), Rotation2d.fromDegrees(40.0))),
    L2(
        new Pose2d(
            new Pose2d(0.0, ReefHeight.L2.height, Rotation2d.fromDegrees(-L2Angle))
                .transformBy(GeomUtil.toTransform2d(tunnelEjectMeters, 0.0))
                .getTranslation(),
            Rotation2d.fromDegrees(L2Angle))),
    L3(
        new Pose2d(
            new Pose2d(0.0, ReefHeight.L3.height, Rotation2d.fromDegrees(-L3Angle))
                .transformBy(GeomUtil.toTransform2d(tunnelEjectMeters, 0.0))
                .getTranslation(),
            Rotation2d.fromDegrees(L3Angle))),
    L4(
        new Pose2d(
            new Pose2d(0.0, ReefHeight.L4.height, Rotation2d.fromDegrees(-L4Angle))
                .transformBy(GeomUtil.toTransform2d(tunnelEjectMeters, 0.0))
                .getTranslation(),
            Rotation2d.fromDegrees(L4Angle))),
    L3_ALGAE(
        new Pose2d(
            new Pose2d(0.0, ReefHeight.L3.height, Rotation2d.fromDegrees(-(L3AngleAlgae + 180.0)))
                .transformBy(GeomUtil.toTransform2d(tunnelEjectMetersReverse, 0.0))
                .getTranslation(),
            Rotation2d.fromDegrees(L3AngleAlgae))),
    L4_ALGAE(
        new Pose2d(
            new Pose2d(0.0, ReefHeight.L4.height, Rotation2d.fromDegrees(-(L4AngleAlgae + 180.0)))
                .transformBy(GeomUtil.toTransform2d(tunnelEjectMetersReverse, 0.0))
                .getTranslation(),
            Rotation2d.fromDegrees(L4AngleAlgae)));

    private final Pose2d pose;

    public double getElevatorHeight() {
      return (pose.getY() - dispenserOrigin2d.getY()) / elevatorAngle.getSin();
    }

    public double getDispenserAngleDeg() {
      return pose.getRotation().getDegrees();
    }
  }

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOW("Stow", 0.0, 0.0),
    L1("L1", 0.15, DispenserPose.L1.getDispenserAngleDeg()),
    L2("L2", DispenserPose.L2.getElevatorHeight(), DispenserPose.L2.getDispenserAngleDeg()),
    L3("L3", DispenserPose.L3.getElevatorHeight(), DispenserPose.L3.getDispenserAngleDeg()),
    L4("L4", DispenserPose.L4.getElevatorHeight(), DispenserPose.L4.getDispenserAngleDeg()),
    ALGAE_FLOOR_INTAKE(
        "AlgaeFloorIntake",
        Units.inchesToMeters(16.5)
            - groundToCarriageZero
            - (pivotToGripper / 2.0 * Rotation2d.fromDegrees(algaeIntakeAngle).getSin()),
        algaeIntakeAngle),
    ALGAE_L2_INTAKE("AlgaeL2Intake", ReefHeight.L2.height - groundToCarriageZero, -30.0),
    ALGAE_L3_INTAKE("AlgaeL3Intake", ReefHeight.L3.height - groundToCarriageZero, -30.0),
    THROW(() -> elevatorMaxTravel, () -> -40.0),
    PRE_PROCESSOR("Processing", 0.05, -80.0),
    ALGAE_STOW("AlgaeStow", 0.0, 25.0),
    ALGAE_STOW_FRONT("AlgaeStowFront", 0.1, pivotSafeAngle.getDegrees()),
    L3_CORAL_REVERSED(
        "L3Reversed",
        DispenserPose.L3_ALGAE.getElevatorHeight(),
        DispenserPose.L3_ALGAE.getDispenserAngleDeg()),
    L4_CORAL_REVERSED(
        "L4Reversed",
        DispenserPose.L4_ALGAE.getElevatorHeight(),
        DispenserPose.L4_ALGAE.getDispenserAngleDeg()),
    L3_CORAL_UNREVERSED("L3Unreversed", DispenserPose.L3_ALGAE.getElevatorHeight(), -40.0),
    L4_CORAL_UNREVERSED("L4Unreversed", DispenserPose.L4_ALGAE.getElevatorHeight(), -40.0);

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
    this(() -> 0.0, () -> Rotation2d.kZero);
  }
}
