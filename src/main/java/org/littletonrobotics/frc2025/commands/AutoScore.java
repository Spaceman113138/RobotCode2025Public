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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.DispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
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
  private static final LoggedTunableNumber linearToleranceEject =
      new LoggedTunableNumber("AutoScore/LinearToleranceEject", 0.05);
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber horizontalL1MaxOffset =
      new LoggedTunableNumber(
          "AutoScore/HorizontalL1MaxOffset", Reef.faceLength / 2 - Units.inchesToMeters(4));
  private static final LoggedTunableNumber verticalL1AlignDistance =
      new LoggedTunableNumber("AutoScore/Vertical1AlignDistance", 0.5);

  private AutoScore() {}

  public static Command getAutoScoreCommand(
      Drive drive,
      Superstructure superstructure,
      Supplier<CoralObjective> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    final DoubleSupplier deadbandedTheta =
        () -> MathUtil.applyDeadband(driverOmega.getAsDouble(), DriveCommands.DEADBAND);
    Supplier<Pose2d> robot = () -> getRobotPose(coralObjective.get(), superstructure.hasAlgae());
    var driveCommand =
        new DriveToPose(
            drive,
            () ->
                coralObjective.get().reefLevel() == FieldConstants.ReefLevel.L1
                    ? AllianceFlipUtil.apply(getL1Pose(robot.get(), coralObjective.get()))
                    : getDriveTarget(
                        robot.get(),
                        AllianceFlipUtil.apply(
                            getCoralScorePose(coralObjective.get(), superstructure.hasAlgae())),
                        superstructure),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () ->
                Math.copySign(
                    deadbandedTheta.getAsDouble() * deadbandedTheta.getAsDouble(),
                    deadbandedTheta.getAsDouble()));
    return Commands.sequence(
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
                          boolean ready = readyToSuperstructure(robot.get(), superstructure);
                          Logger.recordOutput("AutoScore/AllowPreReady", ready);
                          return ready;
                        })
                    .andThen(
                        getSuperstructureAimAndEjectCommand(
                            superstructure,
                            coralObjective,
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
                            })))
            .finallyDo(
                () -> {
                  RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());
                  // Clear logs
                  Logger.recordOutput("AutoScore/AllowPreReady", false);
                  Logger.recordOutput("AutoScore/AllowEject", false);
                }));
  }

  public static Command getReefIntakeCommand(
      Drive drive,
      Superstructure superstructure,
      Supplier<AlgaeObjective> objective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    final DoubleSupplier deadbandedTheta =
        () -> MathUtil.applyDeadband(driverOmega.getAsDouble(), DriveCommands.DEADBAND);
    Supplier<Pose2d> robot = () -> getRobotPose(objective.get());
    return new DriveToPose(
            drive,
            () ->
                getDriveTarget(
                    robot.get(),
                    AllianceFlipUtil.apply(getReefIntakePose(objective.get())),
                    superstructure),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () ->
                Math.copySign(
                    deadbandedTheta.getAsDouble() * deadbandedTheta.getAsDouble(),
                    deadbandedTheta.getAsDouble()))
        .alongWith(
            Commands.sequence(
                Commands.waitUntil(() -> readyToSuperstructure(robot.get(), superstructure)),
                superstructure.runGoal(
                    () ->
                        objective.get().id() % 2 == 0
                            ? SuperstructureState.ALGAE_L3_INTAKE
                            : SuperstructureState.ALGAE_L2_INTAKE)));
  }

  public static Command getSuperstructureAimAndEjectCommand(
      Superstructure superstructure,
      Supplier<CoralObjective> coralObjective,
      BooleanSupplier eject) {
    return Commands.parallel(
        superstructure
            .runGoal(
                () ->
                    Superstructure.getScoringState(
                        coralObjective.get().reefLevel(), superstructure.hasAlgae(), false))
            .until(eject)
            .andThen(
                superstructure
                    .runGoal(
                        () ->
                            Superstructure.getScoringState(
                                coralObjective.get().reefLevel(), superstructure.hasAlgae(), true))
                    .withTimeout(0.2)),
        // Measure distance to branch
        Commands.run(
            () -> {
              var dispenserPose =
                  getRobotPose(coralObjective.get(), superstructure.hasAlgae())
                      .transformBy(
                          GeomUtil.toTransform2d(
                              DispenserPose.fromReefLevel(
                                              coralObjective.get().reefLevel(),
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
            }));
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, Superstructure superstructure) {
    // If superstructure isn't ready move back away from reef
    if (superstructure.hasAlgae() && !superstructure.getState().getValue().isReversed()
        || Superstructure.willSlam(superstructure.getState(), superstructure.getGoal())) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-minDistanceReefClear.get(), 0.0));
    }

    // Flip pose for field constants
    final double distance = robot.getTranslation().getDistance(goal.getTranslation());
    // Flip back to correct alliance
    // Final line up
    double shiftT =
        MathUtil.clamp(
            (distance - Reef.faceLength / 2.0) / (Reef.faceLength * 3.0 - Reef.faceLength / 2.0),
            0.0,
            1.0);
    return goal.transformBy(GeomUtil.toTransform2d(-shiftT * maxDistanceReefLineup.get(), 0.0));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective, boolean algae) {
    var dispenserPose = DispenserPose.fromReefLevel(coralObjective.reefLevel(), algae);
    return getBranchPose(coralObjective) // Use dispenser pose relative to branch to find robot pose
        .transformBy(
            GeomUtil.toTransform2d(
                dispenserPose.getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
                    + dispenserPose.getPose().getX()
                    + SuperstructureConstants.dispenserOrigin2d.getX(),
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  public static Pose2d getReefIntakePose(AlgaeObjective objective) {
    var dispenserPose = DispenserPose.fromAlgaeObjective(objective);
    int branchId = objective.id() * 2;
    return Reef.branchPositions
        .get(branchId)
        .get(FieldConstants.ReefLevel.L3)
        .toPose2d()
        .interpolate(
            Reef.branchPositions.get(branchId + 1).get(FieldConstants.ReefLevel.L3).toPose2d(), 0.5)
        .transformBy(
            GeomUtil.toTransform2d(
                dispenserPose.getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
                    + dispenserPose.getPose().getX()
                    + SuperstructureConstants.dispenserOrigin2d.getX(),
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  private static Pose2d getL1Pose(Pose2d robot, CoralObjective coralObjective) {
    robot = AllianceFlipUtil.apply(robot); // Flip robot onto blue side
    int face = coralObjective.branchId() / 2;
    Transform2d offset = new Transform2d(Reef.centerFaces[face], robot);
    offset =
        new Transform2d(
            verticalL1AlignDistance.get(),
            MathUtil.clamp(
                offset.getY(), -horizontalL1MaxOffset.get(), horizontalL1MaxOffset.get()),
            new Rotation2d(Math.PI));
    return Reef.centerFaces[face].transformBy(offset);
  }

  /** Get whether prescore is allowed without reef interaction. */
  private static boolean readyToSuperstructure(Pose2d robot, Superstructure superstructure) {
    robot = AllianceFlipUtil.apply(robot);
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
    return (withinMaxDistance && withinMinDistance);
  }

  /**
   * Get robot pose based on how far away from associated tag we are. Uses 2d estimation data when
   * close.
   */
  private static Pose2d getRobotPose(CoralObjective coralObjective, boolean hasAlgae) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective, hasAlgae));
  }

  private static Pose2d getRobotPose(AlgaeObjective algaeObjective) {
    return RobotState.getInstance()
        .getReefPose(algaeObjective.id(), getReefIntakePose(algaeObjective));
  }

  private static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions.get(objective.branchId()).get(objective.reefLevel()).toPose2d();
  }
}
