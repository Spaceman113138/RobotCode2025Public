// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoScore extends SequentialCommandGroup {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;

  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", 0.4);
  private static final LoggedTunableNumber maxDistanceSuperstructurePreScore =
      new LoggedTunableNumber(
          "AutoScore/MaxDistanceSuperstructurePreScore", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("AutoScore/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("AutoScore/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber linearToleranceEject =
      new LoggedTunableNumber("AutoScore/LinearToleranceEject", 0.05);
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber horizontalL1MaxOffset =
      new LoggedTunableNumber(
          "AutoScore/HorizontalL1MaxOffset", Reef.faceLength / 2 - Units.inchesToMeters(4));
  private static final LoggedTunableNumber verticalL1AlignDistance =
      new LoggedTunableNumber("AutoScore/Vertical1AlignDistance", 0.5);

  private final Superstructure superstructure;
  private final Supplier<CoralObjective> coralObjective;

  public AutoScore(
      Drive drive,
      Superstructure superstructure,
      Supplier<CoralObjective> coralObjective,
      DoubleSupplier linearX,
      DoubleSupplier linearY,
      DoubleSupplier theta) {
    this.superstructure = superstructure;
    this.coralObjective = coralObjective;

    final DoubleSupplier deadbandedTheta =
        () -> MathUtil.applyDeadband(theta.getAsDouble(), DriveCommands.DEADBAND);
    var driveCommand =
        new DriveToPose(
            drive,
            this::getDriveTarget,
            this::getRobotPose,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        linearX.getAsDouble(), linearY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () ->
                Math.copySign(
                    deadbandedTheta.getAsDouble() * deadbandedTheta.getAsDouble(),
                    deadbandedTheta.getAsDouble()));
    addCommands(
        Commands.runOnce(
            () -> {
              // Clear logs
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            }),
        driveCommand
            .alongWith(
                // Run superstructure
                // Check if need wait until pre ready or already ready
                Commands.waitUntil(
                        () -> {
                          boolean ready = readyToSuperstructureReady();
                          Logger.recordOutput("AutoScore/AllowPreReady", ready);
                          return ready;
                        })
                    .andThen(
                        superstructure
                            .runGoal(
                                () ->
                                    Superstructure.getScoringState(
                                        coralObjective.get().reefLevel(),
                                        superstructure.hasAlgae(),
                                        false))
                            .until(
                                () -> {
                                  boolean ready =
                                      driveCommand.withinTolerance(
                                              linearToleranceEject.get(),
                                              Rotation2d.fromDegrees(thetaToleranceEject.get()))
                                          && superstructure.getState()
                                              == Superstructure.getScoringState(
                                                  coralObjective.get().reefLevel(),
                                                  superstructure.hasAlgae(),
                                                  false);
                                  Logger.recordOutput("AutoScore/AllowEject", ready);
                                  return ready;
                                }),
                        superstructure
                            .runGoal(
                                () ->
                                    Superstructure.getScoringState(
                                        coralObjective.get().reefLevel(),
                                        superstructure.hasAlgae(),
                                        true))
                            .withTimeout(0.2)),

                // Measure distance to branch
                Commands.run(
                    () -> {
                      var dispenserPose =
                          getRobotPose()
                              .transformBy(
                                  GeomUtil.toTransform2d(
                                      getDispenserPose(
                                                      coralObjective.get(),
                                                      superstructure.hasAlgae())
                                                  .getElevatorHeight()
                                              * SuperstructureConstants.elevatorAngle.getCos()
                                          + SuperstructureConstants.dispenserOrigin2d.getX(),
                                      0.0));
                      var offsetTranslation =
                          dispenserPose
                              .relativeTo(
                                  getBranchPose(coralObjective.get())
                                      .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi)))
                              .getTranslation();
                      Logger.recordOutput("AutoScore/DistanceToBranch", -offsetTranslation.getX());
                      RobotState.getInstance()
                          .setDistanceToBranch(
                              offsetTranslation.getNorm() >= 0.6
                                  ? OptionalDouble.empty()
                                  : OptionalDouble.of(-offsetTranslation.getX()));
                    }))
            .finallyDo(
                () -> {
                  RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());
                  // Clear logs
                  Logger.recordOutput("AutoScore/AllowPreReady", false);
                  Logger.recordOutput("AutoScore/AllowEject", false);
                }));
    addRequirements(drive, superstructure);
  }

  /** Get drive target. */
  private Pose2d getDriveTarget() {
    // Get pose of robot aligned to reef
    final Pose2d robot = AllianceFlipUtil.apply(getRobotPose());
    Pose2d reefAlignedPose = getAlignedPose(robot, coralObjective.get(), superstructure.hasAlgae());
    // If superstructure isn't ready move back away from reef
    if (superstructure.hasAlgae()
        && !superstructure.getState().getValue().isReversed()
        && !(coralObjective.get().reefLevel() == FieldConstants.ReefHeight.L1)) {
      reefAlignedPose =
          reefAlignedPose.transformBy(GeomUtil.toTransform2d(-minDistanceReefClear.get(), 0.0));
    }

    // Flip pose for field constants
    final double distance = robot.getTranslation().getDistance(reefAlignedPose.getTranslation());
    // Flip back to correct alliance
    // Final line up
    double shiftT =
        MathUtil.clamp(
            (distance - Reef.faceLength / 2.0) / (Reef.faceLength * 3.0 - Reef.faceLength / 2.0),
            0.0,
            1.0);
    return AllianceFlipUtil.apply(
        reefAlignedPose.transformBy(
            GeomUtil.toTransform2d(-shiftT * maxDistanceReefLineup.get(), 0.0)));
  }

  /** Get whether prescore is allowed without reef interaction. */
  private boolean readyToSuperstructureReady() {
    var robot = AllianceFlipUtil.apply(getRobotPose());
    final double distanceToReefCenter = robot.getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    boolean withinMaxDistance =
        distanceToReefCenter
            <= reefRadius
                + DriveConstants.robotWidth / 2.0
                + maxDistanceSuperstructurePreScore.get();
    Logger.recordOutput("AutoScore/WithinMaxDistance", withinMaxDistance);
    boolean withinMinDistance = distanceToReefCenter >= reefRadius + minDistanceReefClear.get();
    Logger.recordOutput("AutoScore/WithinMinDistance", withinMinDistance);
    return (withinMaxDistance && withinMinDistance)
        || ((superstructure.hasAlgae() && superstructure.getState().getValue().isReversed())
            || (coralObjective.get().reefLevel() == FieldConstants.ReefHeight.L1));
  }

  /**
   * Get robot pose based on how far away from associated tag we are. Uses 2d estimation data when
   * close.
   */
  private Pose2d getRobotPose() {
    final Pose2d finalPose =
        getAlignedPose(
            AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()),
            coralObjective.get(),
            superstructure.hasAlgae());
    var tagPose = RobotState.getInstance().getTxTyPose(getTagID());
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return RobotState.getInstance().getEstimatedPose();
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose().getTranslation())
                        .getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return RobotState.getInstance().getEstimatedPose().interpolate(tagPose.get(), 1.0 - t);
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getAlignedPose(
      Pose2d curPose, CoralObjective coralObjective, boolean algae) {
    if (coralObjective.reefLevel() == FieldConstants.ReefHeight.L1) {
      int face = coralObjective.branchId() / 2;
      Transform2d offset = new Transform2d(Reef.centerFaces[face], curPose);
      offset =
          new Transform2d(
              verticalL1AlignDistance.get(),
              MathUtil.clamp(
                  offset.getY(), -horizontalL1MaxOffset.get(), horizontalL1MaxOffset.get()),
              new Rotation2d(Math.PI));
      return Reef.centerFaces[face].transformBy(offset);
    }

    var dispenserPose = getDispenserPose(coralObjective, algae);
    return getBranchPose(coralObjective) // Use dispenser pose relative to branch to find robot pose
        .transformBy(
            GeomUtil.toTransform2d(
                dispenserPose.getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
                    + dispenserPose.getPose().getX()
                    + SuperstructureConstants.dispenserOrigin2d.getX(),
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  private static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions.get(objective.branchId()).get(objective.reefLevel()).toPose2d();
  }

  private static SuperstructurePose.DispenserPose getDispenserPose(
      CoralObjective coralObjective, boolean algae) {
    return switch (coralObjective.reefLevel()) {
      case L1 -> SuperstructurePose.DispenserPose.L1;
      case L2 -> SuperstructurePose.DispenserPose.L2;
      case L3 ->
          algae ? SuperstructurePose.DispenserPose.L3_ALGAE : SuperstructurePose.DispenserPose.L3;
      case L4 ->
          algae ? SuperstructurePose.DispenserPose.L4_ALGAE : SuperstructurePose.DispenserPose.L4;
    };
  }

  private int getTagID() {
    final boolean isRed = AllianceFlipUtil.shouldFlip();
    return switch (coralObjective.get().branchId()) {
      case 2, 3 -> isRed ? 6 : 19;
      case 4, 5 -> isRed ? 11 : 20;
      case 6, 7 -> isRed ? 10 : 21;
      case 8, 9 -> isRed ? 9 : 22;
      case 10, 11 -> isRed ? 8 : 17;
        // 0, 1
      default -> isRed ? 7 : 18;
    };
  }
}
