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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.*;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.DispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoScore {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClearAlgae =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(18.0));
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(12.0));
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", Units.inchesToMeters(72.0));
  private static final LoggedTunableNumber[] linearXToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.05),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.05),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.05)
  };
  private static final LoggedTunableNumber[] linearYToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.05)
  };
  private static final LoggedTunableNumber[] maxLinearVel = {
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 3)
  };
  private static final LoggedTunableNumber[] maxAngularVel = {
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 3),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 3)
  };
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
                ? getL1Pose(objective)
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
                        objective -> {
                          if (reefLevel.get() == ReefLevel.L1) {
                            return getDriveTarget(
                                robot.get(), AllianceFlipUtil.apply(getL1Pose(objective)));
                          }
                          Pose2d goalPose = getCoralScorePose(objective, superstructure.hasAlgae());
                          if (superstructure.hasAlgae()
                              && !superstructure.getState().getValue().isReversed()) {
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        goalPose.getTranslation().getDistance(Reef.center)
                                            - reefRadius
                                            - (DriveConstants.robotWidth / 2.0)
                                            - minDistanceReefClearAlgae.get(),
                                        0.0));
                          } else if (!superstructure.hasAlgae()
                              && superstructure.getState()
                                  != Superstructure.getScoringState(reefLevel.get(), false, false)
                              && superstructure.getState()
                                  != Superstructure.getScoringState(reefLevel.get(), false, true)) {
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        goalPose.getTranslation().getDistance(Reef.center)
                                            - reefRadius
                                            - (DriveConstants.robotWidth / 2.0)
                                            - minDistanceReefClear.get(),
                                        0.0));
                          }
                          return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                        })
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
        .and(RobotModeTriggers.teleop())
        .onTrue(
            getSuperstructureGetBackCommand(superstructure, superstructure::getState)
                .andThen(() -> needsToGetBack.value = false));

    return Commands.runOnce(
            () -> {
              // Start LEDs
              Leds.getInstance().autoScoring = true;
              Leds.getInstance().autoScoringLevel = reefLevel.get();

              // Reset state
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
                  boolean ready =
                      withinDistanceToReef(robot.get(), distanceSuperstructureReady.get());
                  Logger.recordOutput("AutoScore/AllowPreReady", ready);

                  // Get back!
                  if (ready) {
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

                  int intReefLevel = coralObjective.get().get().reefLevel().ordinal();
                  boolean ready =
                      Math.abs(poseError.getTranslation().getX())
                              <= linearXToleranceEject[intReefLevel].get()
                          && Math.abs(poseError.getTranslation().getY())
                              <= linearYToleranceEject[intReefLevel].get()
                          && Math.hypot(
                                  RobotState.getInstance().getRobotVelocity().vxMetersPerSecond,
                                  RobotState.getInstance().getRobotVelocity().vyMetersPerSecond)
                              <= maxLinearVel[intReefLevel].get()
                          && Math.abs(
                                  RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond)
                              <= maxAngularVel[intReefLevel].get()
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

              // Stop LEDs
              Leds.getInstance().autoScoring = false;

              // Indicate has ended command
              hasEnded.value = true;
            });
  }

  public static Command getAutoScoreCommand(
      Drive drive,
      Superstructure superstructure,
      Function<Supplier<CoralObjective>, Command> requestCoralScoredCommand,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return getAutoScoreCommand(
        drive,
        superstructure,
        requestCoralScoredCommand,
        reefLevel,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0);
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

    Supplier<Pose2d> goal =
        () -> {
          Pose2d goalPose =
              algaeObjective
                  .get()
                  .map(objective -> AllianceFlipUtil.apply(getReefIntakePose(objective)))
                  .orElseGet(
                      () ->
                          algaeIntaked.value == null
                              ? robot.get()
                              : AllianceFlipUtil.apply(getReefIntakePose(algaeIntaked.value)));
          if (algaeObjective.get().isEmpty() && algaeIntaked.value == null) {
            return goalPose;
          }
          if (superstructure.hasAlgae()) {
            return goalPose.transformBy(
                GeomUtil.toTransform2d(-minDistanceReefClearAlgae.get(), 0.0));
          }
          return goalPose;
        };

    // Schedule get back command
    new Trigger(() -> hasEnded.value && needsToGetBack.value)
        .and(RobotModeTriggers.teleop())
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
                () -> getDriveTarget(robot.get(), goal.get()),
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
            superstructure
                .runGoal(algaeIntakeState)
                .alongWith(
                    Commands.waitUntil(() -> superstructure.getState() == algaeIntakeState.get()),
                    Commands.runOnce(() -> needsToGetBack.value = true)),
            Commands.waitUntil(superstructure::hasAlgae)
                .andThen(
                    Commands.runOnce(
                            () -> {
                              algaeIntaked.value = algaeObjective.get().orElse(null);
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
                                            DispenserPose.forCoralScore(
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
                superstructure.hasAlgae()
                    ? outOfDistanceToReef(
                        RobotState.getInstance().getEstimatedPose(),
                        minDistanceReefClearAlgae.get())
                    : outOfDistanceToReef(
                        RobotState.getInstance().getEstimatedPose(), minDistanceReefClear.get()))
        .withName("Superstructure Get Back!");
  }

  /** Get drive target. */
  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineup.get(),
            Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective, boolean algae) {
    return getBranchPose(coralObjective)
        .transformBy(DispenserPose.forCoralScore(coralObjective.reefLevel(), algae).toRobotPose());
  }

  public static Pose2d getReefIntakePose(AlgaeObjective objective) {
    int branchId = objective.id() * 2;
    return getBranchPose(new CoralObjective(branchId, ReefLevel.L3))
        .interpolate(getBranchPose(new CoralObjective(branchId + 1, ReefLevel.L3)), 0.5)
        .transformBy(DispenserPose.forAlgaeIntake(objective).toRobotPose());
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    int face = coralObjective.branchId() / 2;
    return Reef.centerFaces[face].transformBy(
        new Transform2d(
            l1AlignOffsetX.get(),
            l1AlignOffsetY.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0),
            Rotation2d.fromDegrees(
                l1AlignOffsetDegrees.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0))));
  }

  private static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= reefRadius + DriveConstants.robotWidth / 2.0 + distance;
  }

  private static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= reefRadius + DriveConstants.robotWidth / 2.0 + distance;
  }

  private static Pose2d getRobotPose(CoralObjective coralObjective, boolean hasAlgae) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective, hasAlgae));
  }

  private static Pose2d getRobotPose(AlgaeObjective algaeObjective) {
    return RobotState.getInstance()
        .getReefPose(algaeObjective.id(), getReefIntakePose(algaeObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
  }
}
