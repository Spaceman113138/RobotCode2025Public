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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToAlgae extends DriveToPose {
  private static final LoggedTunableNumber lookAheadSecs =
      new LoggedTunableNumber("DriveToAlgae/LookAheadSecs", 0.3);
  private static final LoggedTunableNumber angleDifferenceWeight =
      new LoggedTunableNumber("DriveToAlgae/AngleDifferenceWeight", 0.3);

  public DriveToAlgae(
      Drive drive, DoubleSupplier linearX, DoubleSupplier linearY, DoubleSupplier theta) {
    super(
        drive,
        () -> {
          RobotState instance = RobotState.getInstance();
          Pose2d currentPose = instance.getEstimatedPose();

          ChassisSpeeds robotVelocity = instance.getRobotVelocity();
          Translation2d linearFieldVelocity =
              new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

          Pose2d lookAheadPose =
              instance
                  .getEstimatedPose()
                  .transformBy(
                      new Transform2d(linearFieldVelocity, new Rotation2d())
                          .times(lookAheadSecs.get()));
          Logger.recordOutput("DriveToAlgae/LookAheadPose", lookAheadPose);

          double closestValue = 999;

          Pose2d closestAlgae = new Pose2d();

          for (var algae : instance.getAlgaeTranslations()) {
            if (algae == null) {
              closestAlgae = new Pose2d(algae, new Rotation2d());
            } else {
              double value =
                  algae.getDistance(lookAheadPose.getTranslation())
                      + Math.abs(
                          algae
                                  .minus(currentPose.getTranslation())
                                  .getAngle()
                                  .minus(currentPose.getRotation())
                                  .getRadians()
                              * angleDifferenceWeight.get());
              if (value < closestValue) {
                closestAlgae =
                    new Pose2d(algae, algae.minus(currentPose.getTranslation()).getAngle());
                closestValue = value;
              }
            }
          }
          Logger.recordOutput("DriveToAlgae/ClosestAlgae", closestAlgae);

          return closestAlgae;
        },
        RobotState.getInstance()::getEstimatedPose,
        () ->
            DriveCommands.getLinearVelocityFromJoysticks(
                    linearX.getAsDouble(), linearY.getAsDouble())
                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
        () ->
            Math.copySign(
                Math.pow(MathUtil.applyDeadband(theta.getAsDouble(), DriveCommands.DEADBAND), 2.0),
                theta.getAsDouble()));
  }
}
