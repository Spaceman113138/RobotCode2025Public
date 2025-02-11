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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.DispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ReefIntake extends SequentialCommandGroup {
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("ReefIntake/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("ReefIntake/MinDistanceReefClear", 0.4);
  private static final LoggedTunableNumber maxDistanceSuperstructurePreScore =
      new LoggedTunableNumber(
          "ReefIntake/MaxDistanceSuperstructurePreReady", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber ffInputDeadband =
      new LoggedTunableNumber("ReefIntake/FFInputDeadband", 0.2);
  private static final LoggedTunableNumber maxLinearFFVelocity =
      new LoggedTunableNumber("ReefIntake/MaxLinearFF", Units.feetToMeters(7.0));
  private static final LoggedTunableNumber maxThetaFFVelocity =
      new LoggedTunableNumber("ReefIntake/MaxThetaFF", Units.degreesToRadians(270.0));

  private final Superstructure superstructure;
  private final Supplier<AlgaeObjective> objective;

  public ReefIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<AlgaeObjective> objective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    this.superstructure = superstructure;
    this.objective = objective;

    final DoubleSupplier deadbandedOmega =
        () -> MathUtil.applyDeadband(driverOmega.getAsDouble(), ffInputDeadband.get());
    addCommands(
        new DriveToPose(
                drive,
                this::getDriveTarget,
                this::getRobotPose,
                () ->
                    DriveCommands.getLinearVelocityFromJoysticks(
                            driverX.getAsDouble(), driverY.getAsDouble())
                        .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                () ->
                    Math.copySign(
                        deadbandedOmega.getAsDouble() * deadbandedOmega.getAsDouble(),
                        deadbandedOmega.getAsDouble()))
            .alongWith(
                Commands.sequence(
                    Commands.waitUntil(this::readyToSuperstructure),
                    superstructure.runGoal(
                        () ->
                            objective.get().id() % 2 == 0
                                ? SuperstructureState.ALGAE_L3_INTAKE
                                : SuperstructureState.ALGAE_L2_INTAKE))));
  }

  private Pose2d getDriveTarget() {
    // Get pose of robot aligned to reef
    Pose2d alignedPose = getAlignedPose(objective.get());
    Logger.recordOutput("aligned", alignedPose);
    // If superstructure isn't ready move back away from reef
    var state = superstructure.getState();
    if (state.getValue().getHeight().lowerThan(SuperstructureStateData.Height.INTAKE)) {
      alignedPose =
          alignedPose.transformBy(GeomUtil.toTransform2d(-minDistanceReefClear.get(), 0.0));
    }

    // Flip pose for field constants
    final Pose2d robot = AllianceFlipUtil.apply(getRobotPose());
    final double distance = robot.getTranslation().getDistance(alignedPose.getTranslation());
    // Flip back to correct alliance
    // Final line up
    double shiftT =
        MathUtil.clamp(
            (distance - Reef.faceLength / 2.0) / (Reef.faceLength * 3.0 - Reef.faceLength / 2.0),
            0.0,
            1.0);
    return AllianceFlipUtil.apply(
        alignedPose.transformBy(
            GeomUtil.toTransform2d(-shiftT * maxDistanceReefLineup.get(), 0.0)));
  }

  /** Get whether prescore is allowed without reef interaction. */
  private boolean readyToSuperstructure() {
    var robot = AllianceFlipUtil.apply(getRobotPose());
    final double distanceToReefCenter = robot.getTranslation().getDistance(Reef.center);
    return (distanceToReefCenter
        <= reefRadius + DriveConstants.robotWidth / 2.0 + maxDistanceSuperstructurePreScore.get());
  }

  public static Pose2d getAlignedPose(AlgaeObjective objective) {
    var dispenserPose = getDispenserPose(objective);
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

  private static DispenserPose getDispenserPose(AlgaeObjective objective) {
    if (objective.id() % 2 == 0) {
      return DispenserPose.L3_ALGAE_INTAKE;
    } else {
      return DispenserPose.L2_ALGAE_INTAKE;
    }
  }

  private Pose2d getRobotPose() {
    return RobotState.getInstance().getReefPose(getTagID(), getAlignedPose(objective.get()));
  }

  private int getTagID() {
    final boolean isRed = AllianceFlipUtil.shouldFlip();
    return switch (objective.get().id()) {
      case 1 -> isRed ? 6 : 19;
      case 2 -> isRed ? 11 : 20;
      case 3 -> isRed ? 10 : 21;
      case 4 -> isRed ? 9 : 22;
      case 5 -> isRed ? 8 : 17;
        // 0
      default -> isRed ? 7 : 18;
    };
  }
}
