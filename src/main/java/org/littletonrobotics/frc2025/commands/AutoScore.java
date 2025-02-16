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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.DispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoScore extends SequentialCommandGroup {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", 0.3);
  private static final LoggedTunableNumber maxDistanceSuperstructurePreScore =
      new LoggedTunableNumber(
          "AutoScore/MaxDistanceSuperstructurePreScore", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber linearToleranceEject =
      new LoggedTunableNumber("AutoScore/LinearToleranceEject", 0.05);
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.5);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.3);
  private static final LoggedTunableNumber l1AlignOffsetDegrees =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetDegrees", 170.0);
  private static final LoggedTunableNumber maxDistanceAim =
      new LoggedTunableNumber("AutoScore/MaxDistanceAim", 0.6);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber ejectTimeSeconds =
      new LoggedTunableNumber("AutoScore/EjectTimeSeconds", 0.3);

  private AutoScore() {}

  public static Command getAutoScoreCommand(
      Drive drive,
      Superstructure superstructure,
      Function<Supplier<CoralObjective>, Command> requestCoralScoredCommand,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    final DoubleSupplier deadbandedTheta =
        () -> MathUtil.applyDeadband(driverOmega.getAsDouble(), DriveCommands.DEADBAND);
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(objective -> getRobotPose(objective, superstructure.hasAlgae()))
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());
    Function<CoralObjective, Pose2d> goal =
        objective ->
            objective.reefLevel() == ReefLevel.L1
                ? getL1Pose(robot.get(), objective)
                : getCoralScorePose(objective, superstructure.hasAlgae());
    Container<CoralObjective> coralObjectiveScored = new Container<>();
    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    var driveCommand =
        new DriveToPose(
            drive,
            () ->
                coralObjective
                    .get()
                    .map(
                        objective ->
                            getDriveTarget(
                                robot.get(),
                                AllianceFlipUtil.apply(goal.apply(objective)),
                                superstructure))
                    .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () ->
                Math.copySign(
                    deadbandedTheta.getAsDouble() * deadbandedTheta.getAsDouble(),
                    deadbandedTheta.getAsDouble()));

    // Schedule get back command
    new Trigger(() -> hasEnded.value && needsToGetBack.value)
        .onTrue(
            getSuperstructureGetBackCommand(
                    superstructure,
                    () ->
                        Superstructure.getScoringState(
                            reefLevel.get(), superstructure.hasAlgae(), false))
                .andThen(() -> needsToGetBack.value = false));

    return Commands.runOnce(
            () -> {
              // Reset State
              needsToGetBack.value = false;
              hasEnded.value = false;

              // Clear logs
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            // Run superstructure
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready = readyToSuperstructure(robot.get(), superstructure);
                  Logger.recordOutput("AutoScore/AllowPreReady", ready);

                  // Get back!
                  if (ready && superstructure.hasAlgae()) {
                    needsToGetBack.value = true;
                  }

                  return ready;
                }),
            getSuperstructureAimAndEjectCommand(
                superstructure,
                reefLevel,
                coralObjective,
                () -> {
                  if (coralObjective.get().isEmpty()) return false;
                  Pose2d poseError =
                      AllianceFlipUtil.apply(robot.get())
                          .relativeTo(goal.apply(coralObjective.get().get()));
                  boolean ready =
                      poseError.getTranslation().getNorm() <= linearToleranceEject.get()
                          && Math.abs(poseError.getRotation().getDegrees())
                              <= thetaToleranceEject.get()
                          && superstructure.atGoal();
                  Logger.recordOutput("AutoScore/AllowEject", ready);
                  if (ready) {
                    coralObjectiveScored.value = coralObjective.get().get();
                  }
                  return ready;
                }),
            requestCoralScoredCommand.apply(() -> coralObjectiveScored.value))
        .deadlineFor(driveCommand) // Deadline with driving command
        .finallyDo(
            interrupted -> {
              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());

              // Clear logs
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);

              // Indicate has ended command
              hasEnded.value = true;
            });
  }

  public static Command getReefIntakeCommand(
      Drive drive,
      Superstructure superstructure,
      Function<Supplier<AlgaeObjective>, Command> requestAlgaeIntakedCommand,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    final DoubleSupplier deadbandedTheta =
        () -> MathUtil.applyDeadband(driverOmega.getAsDouble(), DriveCommands.DEADBAND);
    Supplier<Pose2d> robot =
        () ->
            algaeObjective
                .get()
                .map(AutoScore::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());

    Supplier<SuperstructureState> algaeIntakeState =
        () ->
            algaeObjective
                .get()
                .map(
                    objective ->
                        objective.id() % 2 == 0
                            ? SuperstructureState.ALGAE_L3_INTAKE
                            : SuperstructureState.ALGAE_L2_INTAKE)
                .orElseGet(superstructure::getState);

    Container<AlgaeObjective> algaeIntaked = new Container<>();
    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    // Schedule get back command
    new Trigger(() -> hasEnded.value && needsToGetBack.value)
        .onTrue(
            getSuperstructureGetBackCommand(superstructure, algaeIntakeState)
                .andThen(() -> needsToGetBack.value = false));

    return Commands.runOnce(
            () -> {
              // Reset State
              algaeIntaked.value = null;
              needsToGetBack.value = false;
              hasEnded.value = false;
            })
        .andThen(
            new DriveToPose(
                drive,
                () ->
                    algaeObjective
                        .get()
                        .map(
                            objective ->
                                getDriveTarget(
                                    robot.get(),
                                    AllianceFlipUtil.apply(getReefIntakePose(objective)),
                                    superstructure))
                        .orElseGet(
                            () ->
                                algaeIntaked.value == null
                                    ? robot.get()
                                    : getDriveTarget(
                                        robot.get(),
                                        AllianceFlipUtil.apply(
                                            getReefIntakePose(algaeIntaked.value)),
                                        superstructure)),
                robot,
                () ->
                    DriveCommands.getLinearVelocityFromJoysticks(
                            driverX.getAsDouble(), driverY.getAsDouble())
                        .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                () ->
                    Math.copySign(
                        deadbandedTheta.getAsDouble() * deadbandedTheta.getAsDouble(),
                        deadbandedTheta.getAsDouble())))
        .alongWith(
            superstructure.runGoal(algaeIntakeState),
            Commands.waitUntil(superstructure::hasAlgae)
                .andThen(
                    Commands.runOnce(
                            () -> {
                              needsToGetBack.value = true;
                              algaeIntaked.value = algaeObjective.get().get();
                            })
                        .andThen(requestAlgaeIntakedCommand.apply(() -> algaeIntaked.value))
                        .onlyIf(() -> algaeObjective.get().isPresent())))
        .finallyDo(() -> hasEnded.value = true);
  }

  public static Command getSuperstructureAimAndEjectCommand(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject) {
    final Timer ejectTimer = new Timer();
    return superstructure
        .runGoal(
            () -> Superstructure.getScoringState(reefLevel.get(), superstructure.hasAlgae(), false))
        .until(eject)
        .andThen(
            Commands.runOnce(ejectTimer::restart),
            superstructure
                .runGoal(
                    () ->
                        Superstructure.getScoringState(
                            reefLevel.get(), superstructure.hasAlgae(), true))
                .until(() -> ejectTimer.hasElapsed(ejectTimeSeconds.get())))
        .deadlineFor(
            // Measure distance to branch
            Commands.run(
                () -> {
                  coralObjective
                      .get()
                      .ifPresentOrElse(
                          objective -> {
                            var dispenserPose =
                                getRobotPose(objective, superstructure.hasAlgae())
                                    .transformBy(
                                        GeomUtil.toTransform2d(
                                            DispenserPose.fromReefLevel(
                                                            objective.reefLevel(),
                                                            superstructure.hasAlgae())
                                                        .getElevatorHeight()
                                                    * SuperstructureConstants.elevatorAngle.getCos()
                                                + SuperstructureConstants.dispenserOrigin2d.getX(),
                                            0.0));
                            var offsetTranslation =
                                dispenserPose
                                    .relativeTo(
                                        getBranchPose(objective)
                                            .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi)))
                                    .getTranslation();
                            double distanceToBranch = -offsetTranslation.getX();
                            Logger.recordOutput("AutoScore/DistanceToBranch", distanceToBranch);
                            RobotState.getInstance()
                                .setDistanceToBranch(
                                    offsetTranslation.getNorm() >= maxDistanceAim.get()
                                            || distanceToBranch <= minDistanceAim.get()
                                        ? OptionalDouble.empty()
                                        : OptionalDouble.of(distanceToBranch));
                          },
                          () ->
                              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty()));
                }));
  }

  private static Command getSuperstructureGetBackCommand(
      Superstructure superstructure, Supplier<SuperstructureState> holdState) {
    return superstructure
        .runGoal(holdState)
        .until(
            () ->
                AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose())
                        .getTranslation()
                        .getDistance(FieldConstants.Reef.center)
                    >= FieldConstants.Reef.faceLength
                        + DriveConstants.robotWidth / 2.0
                        + AutoScore.minDistanceReefClear.get())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Superstructure Get Back!");
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal, Superstructure superstructure) {
    // If superstructure isn't ready move back away from reef
    if (superstructure.hasAlgae() && !superstructure.getState().getValue().isReversed()) {
      goal = goal.transformBy(GeomUtil.toTransform2d(-minDistanceReefClear.get(), 0.0));
    }

    // Final line up
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT = MathUtil.clamp(offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineup.get(),
            Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
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
        .get(ReefLevel.L3)
        .toPose2d()
        .interpolate(Reef.branchPositions.get(branchId + 1).get(ReefLevel.L3).toPose2d(), 0.5)
        .transformBy(
            GeomUtil.toTransform2d(
                dispenserPose.getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
                    + dispenserPose.getPose().getX()
                    + SuperstructureConstants.dispenserOrigin2d.getX(),
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  private static Pose2d getL1Pose(Pose2d robot, CoralObjective coralObjective) {
    int face = coralObjective.branchId() / 2;
    return Reef.centerFaces[face].transformBy(
        new Transform2d(
            l1AlignOffsetX.get(),
            l1AlignOffsetY.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0),
            Rotation2d.fromDegrees(
                l1AlignOffsetDegrees.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0))));
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
    return withinMaxDistance;
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

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions.get(objective.branchId()).get(objective.reefLevel()).toPose2d();
  }
}
