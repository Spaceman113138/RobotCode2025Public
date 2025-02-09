// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants.ReefHeight;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final double groundToCarriageZero = dispenserOrigin2d.getY();
  private static final double tunnelEjectMeters = Units.inchesToMeters(5.0);
  private static final double tunnelEjectMetersReverse = Units.inchesToMeters(8.0);

  private static final Map<ReefHeight, Pair<Double, Double>> ejectMeters =
      Map.of(
          ReefHeight.L1,
          Pair.of(Units.inchesToMeters(15.0), Units.inchesToMeters(15.0)),
          ReefHeight.L2,
          Pair.of(Units.inchesToMeters(5.0), Units.inchesToMeters(12.0)),
          ReefHeight.L3,
          Pair.of(Units.inchesToMeters(5.0), Units.inchesToMeters(12.0)),
          ReefHeight.L4,
          Pair.of(Units.inchesToMeters(5.0), Units.inchesToMeters(12.0)));
  private static final Map<ReefHeight, Pair<Double, Double>> optimalCoralAngles =
      Map.of(
          ReefHeight.L1,
          //     No Algae  Algae (Need to flip will reverse)
          Pair.of(40.0, -40.0),
          ReefHeight.L2,
          Pair.of(-35.0, -30.0),
          ReefHeight.L3,
          Pair.of(-35.0, -30.0),
          ReefHeight.L4,
          Pair.of(-25.0, -25.0));

  @Getter
  @RequiredArgsConstructor
  /**
   * Dispenser pose relative to branch on ground (looking at the robot from the left side) Rotation
   * is just the rotation of dispenser when scoring.
   */
  public enum DispenserPose {
    L1(
        new Pose2d(
            Units.inchesToMeters(15.0),
            dispenserOrigin2d.getY(),
            Rotation2d.fromDegrees(optimalCoralAngles.get(ReefHeight.L1).getFirst()))),
    L2(ReefHeight.L2, false),
    L3(ReefHeight.L3, false),
    L4(ReefHeight.L4, false),
    L1_ALGAE(ReefHeight.L1, true),
    L2_ALGAE(ReefHeight.L2, true),
    L3_ALGAE(ReefHeight.L3, true),
    L4_ALGAE(ReefHeight.L4, true);

    private ReefHeight reefHeight = ReefHeight.L1;
    private boolean algae = false;
    private final Pose2d pose;

    DispenserPose(ReefHeight reefHeight, boolean algae) {
      this.reefHeight = reefHeight;
      this.algae = algae;
      // Calculate dispenser pose relative to appropriate branch
      double dispenserAngleDeg =
          algae
              ? optimalCoralAngles.get(reefHeight).getSecond()
              : optimalCoralAngles.get(reefHeight).getFirst();
      pose =
          new Pose2d(
              new Pose2d(0.0, reefHeight.height, Rotation2d.fromDegrees(-dispenserAngleDeg))
                  .transformBy(
                      GeomUtil.toTransform2d(
                          (algae
                                  ? ejectMeters.get(reefHeight).getSecond()
                                  : ejectMeters.get(reefHeight).getFirst())
                              + (algae ? pivotToTunnelBack : pivotToTunnelFront),
                          0.0))
                  .getTranslation(),
              Rotation2d.fromDegrees(algae ? dispenserAngleDeg - 180.0 : dispenserAngleDeg));
    }

    public double getElevatorHeight() {
      return (pose.getY() - dispenserOrigin2d.getY()) / elevatorAngle.getSin();
    }

    public double getDispenserAngleDeg() {
      return pose.getRotation().getDegrees();
    }

    public static DispenserPose fromReefHeight(ReefHeight reefHeight, boolean algae) {
      return Arrays.stream(values())
          .filter(
              dispenserPose ->
                  dispenserPose.reefHeight == reefHeight && dispenserPose.algae == algae)
          .findFirst()
          .orElse(L1);
    }
  }

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOW("Stow", 0.0, 10.0),
    L1("L1", 0.15, DispenserPose.L1.getDispenserAngleDeg()),
    L2(ReefHeight.L2, false),
    L3(ReefHeight.L3, false),
    L4(ReefHeight.L4, false),
    ALGAE_FLOOR_INTAKE("AlgaeFloorIntake", 0.4, -20.0),
    ALGAE_L2_INTAKE("AlgaeL2Intake", ReefHeight.L2.height - groundToCarriageZero, -30.0),
    ALGAE_L3_INTAKE("AlgaeL3Intake", ReefHeight.L3.height - groundToCarriageZero, -30.0),
    THROW(() -> elevatorMaxTravel, () -> 40.0),
    PRE_PROCESSOR("Processing", 0.2, -10.0),
    ALGAE_STOW(
        "AlgaeStow",
        stageHeight - dispenserToCarriage - bottomToDispenser,
        pivotSafeAngle.getDegrees()),
    L1_CORAL_REVERSED(ReefHeight.L1, true),
    L2_CORAL_REVERSED(ReefHeight.L2, true),
    L3_CORAL_REVERSED(ReefHeight.L3, true),
    L4_CORAL_REVERSED(ReefHeight.L4, true);

    private final SuperstructurePose pose;

    Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromDegrees(pivotAngle.getAsDouble())));
    }

    Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }

    Preset(ReefHeight reefHeight, boolean algae) {
      var dispenserPose = DispenserPose.fromReefHeight(reefHeight, algae);
      var elevatorHeight =
          new LoggedTunableNumber(
              "Superstructure/" + reefHeight + (algae ? "Algae" : "") + "/Elevator",
              dispenserPose.getElevatorHeight());
      pose =
          new SuperstructurePose(
              elevatorHeight,
              () ->
                  RobotState.getInstance().getDistanceToBranch().isEmpty()
                      ? Rotation2d.fromDegrees(dispenserPose.getDispenserAngleDeg())
                      : Rotation2d.fromDegrees(
                          new Rotation2d(
                                      RobotState.getInstance().getDistanceToBranch().getAsDouble(),
                                      reefHeight.height
                                          - (elevatorHeight.get() * elevatorAngle.getSin()
                                              + dispenserOrigin2d.getY()))
                                  .getDegrees()
                              + (algae ? -180.0 : 0.0)));
    }
  }

  SuperstructurePose() {
    this(() -> 0.0, () -> Rotation2d.kZero);
  }
}
