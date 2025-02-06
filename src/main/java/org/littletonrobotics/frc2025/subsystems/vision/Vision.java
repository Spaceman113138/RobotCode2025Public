// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import static org.littletonrobotics.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.RobotState.AlgaeTxTyObservation;
import org.littletonrobotics.frc2025.RobotState.TxTyObservation;
import org.littletonrobotics.frc2025.RobotState.VisionObservation;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Vision subsystem for vision. */
@ExtensionMethod({GeomUtil.class})
public class Vision extends VirtualSubsystem {
  private final Supplier<AprilTagLayoutType> aprilTagLayoutSupplier;
  private final VisionIO[] io;
  private final AprilTagVisionIOInputsAutoLogged[] aprilTagInputs;
  private final ObjDetectVisionIOInputsAutoLogged[] objDetectInputs;
  private final LoggedNetworkBoolean recordingRequest =
      new LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private Pose3d demoTagPose = null;
  private double lastDemoTagPoseTimestamp = 0.0;

  public Vision(Supplier<AprilTagLayoutType> aprilTagLayoutSupplier, VisionIO... io) {
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
    this.io = io;
    aprilTagInputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    objDetectInputs = new ObjDetectVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      aprilTagInputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }
    for (int i = 0; i < io.length; i++) {
      objDetectInputs[i] = new ObjDetectVisionIOInputsAutoLogged();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(aprilTagInputs[i], objDetectInputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, aprilTagInputs[i]);
      Logger.processInputs("ObjDetectVision/Inst" + i, objDetectInputs[i]);
    }

    // Update recording state
    boolean shouldRecord = DriverStation.isFMSAttached() || recordingRequest.get();
    for (var ioInst : io) {
      ioInst.setRecording(shouldRecord);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    List<AlgaeTxTyObservation> allAlgaeTxTyObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      for (int frameIndex = 0;
          frameIndex < aprilTagInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getTimestamp());
        var timestamp =
            aprilTagInputs[instanceIndex].timestamps[frameIndex] + timestampOffset.get();
        var values = aprilTagInputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d =
                cameraPose.transformBy(
                    cameras[instanceIndex].pose().get().toTransform3d().inverse());
            useVisionRotation = true;
            break;
          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Transform3d cameraToRobot =
                cameras[instanceIndex].pose().get().toTransform3d().inverse();
            Pose3d robotPose3d0 = cameraPose0.transformBy(cameraToRobot);
            Pose3d robotPose3d1 = cameraPose1.transformBy(cameraToRobot);

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
              Rotation2d currentRotation = RobotState.getInstance().getRotation();
              Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose3d = robotPose3d0;
              } else {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              }
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }

        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
          Optional<Pose3d> tagPose =
              aprilTagLayoutSupplier.get().getLayout().getTagPose((int) values[i]);
          tagPose.ifPresent(tagPoses::add);
        }
        if (tagPoses.isEmpty()) continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStdDev =
            xyStdDevCoefficient
                * Math.pow(avgDistance, 2.0)
                / tagPoses.size()
                * cameras[instanceIndex].stdDevFactor();
        double thetaStdDev =
            useVisionRotation
                ? thetaStdDevCoefficient
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.size()
                    * cameras[instanceIndex].stdDevFactor()
                : Double.POSITIVE_INFINITY;
        allVisionObservations.add(
            new VisionObservation(
                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        // Log data from instance
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getTimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      }

      // Get tag tx ty observation data
      Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();
      for (int frameIndex = 0;
          frameIndex < aprilTagInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        var timestamp = aprilTagInputs[instanceIndex].timestamps[frameIndex];
        var values = aprilTagInputs[instanceIndex].frames[frameIndex];
        int tagEstimationDataEndIndex =
            switch ((int) values[0]) {
              default -> 0;
              case 1 -> 8;
              case 2 -> 16;
            };

        for (int index = tagEstimationDataEndIndex + 1; index < values.length; index += 10) {
          double[] tx = new double[4];
          double[] ty = new double[4];
          for (int i = 0; i < 4; i++) {
            tx[i] = values[index + 1 + (2 * i)];
            ty[i] = values[index + 1 + (2 * i) + 1];
          }
          int tagId = (int) values[index];
          double distance = values[index + 9];

          txTyObservations.put(
              tagId, new TxTyObservation(tagId, instanceIndex, tx, ty, distance, timestamp));
        }
      }

      // Save tx ty observation data
      for (var observation : txTyObservations.values()) {
        if (!allTxTyObservations.containsKey(observation.tagId())
            || observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
          allTxTyObservations.put(observation.tagId(), observation);
        }
      }

      // Record algae observations
      for (int frameIndex = 0;
          frameIndex < objDetectInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        double[] frame = objDetectInputs[instanceIndex].frames[frameIndex];
        for (int i = 0; i < frame.length; i += 10) {
          if (frame[i + 1] > objDetectConfidenceThreshold) {
            double[] tx = new double[4];
            double[] ty = new double[4];
            for (int z = 0; z < 4; z++) {
              tx[z] = frame[i + 2 + (2 * z)];
              ty[z] = frame[i + 2 + (2 * z) + 1];
            }
            allAlgaeTxTyObservations.add(
                new AlgaeTxTyObservation(
                    instanceIndex, tx, ty, objDetectInputs[instanceIndex].timestamps[frameIndex]));
          }
        }
      }

      // Record demo tag pose
      if (aprilTagInputs[instanceIndex].demoFrame.length > 0) {
        var values = aprilTagInputs[instanceIndex].demoFrame;
        double error0 = values[0];
        double error1 = values[8];
        Pose3d fieldToCameraPose =
            new Pose3d(RobotState.getInstance().getEstimatedPose())
                .transformBy(cameras[instanceIndex].pose().get().toTransform3d());
        Pose3d fieldToTagPose0 =
            fieldToCameraPose.transformBy(
                new Transform3d(
                    new Translation3d(values[1], values[2], values[3]),
                    new Rotation3d(new Quaternion(values[4], values[5], values[6], values[7]))));
        Pose3d fieldToTagPose1 =
            fieldToCameraPose.transformBy(
                new Transform3d(
                    new Translation3d(values[9], values[10], values[11]),
                    new Rotation3d(
                        new Quaternion(values[12], values[13], values[14], values[15]))));
        Pose3d fieldToTagPose;

        // Find best pose
        if (demoTagPose == null && error0 < error1) {
          fieldToTagPose = fieldToTagPose0;
        } else if (demoTagPose == null && error0 >= error1) {
          fieldToTagPose = fieldToTagPose1;
        } else if (error0 < error1 * ambiguityThreshold) {
          fieldToTagPose = fieldToTagPose0;
        } else if (error1 < error0 * ambiguityThreshold) {
          fieldToTagPose = fieldToTagPose1;
        } else {
          var pose0Quaternion = fieldToTagPose0.getRotation().getQuaternion();
          var pose1Quaternion = fieldToTagPose1.getRotation().getQuaternion();
          var referenceQuaternion = demoTagPose.getRotation().getQuaternion();
          double pose0Distance =
              Math.acos(
                  pose0Quaternion.getW() * referenceQuaternion.getW()
                      + pose0Quaternion.getX() * referenceQuaternion.getX()
                      + pose0Quaternion.getY() * referenceQuaternion.getY()
                      + pose0Quaternion.getZ() * referenceQuaternion.getZ());
          double pose1Distance =
              Math.acos(
                  pose1Quaternion.getW() * referenceQuaternion.getW()
                      + pose1Quaternion.getX() * referenceQuaternion.getX()
                      + pose1Quaternion.getY() * referenceQuaternion.getY()
                      + pose1Quaternion.getZ() * referenceQuaternion.getZ());
          if (pose0Distance > Math.PI / 2) {
            pose0Distance = Math.PI - pose0Distance;
          }
          if (pose1Distance > Math.PI / 2) {
            pose1Distance = Math.PI - pose1Distance;
          }
          if (pose0Distance < pose1Distance) {
            fieldToTagPose = fieldToTagPose0;
          } else {
            fieldToTagPose = fieldToTagPose1;
          }
        }

        // Save pose
        if (fieldToTagPose != null) {
          demoTagPose = fieldToTagPose;
          lastDemoTagPoseTimestamp = Timer.getTimestamp();
        }

        // If no frames from instances, clear robot pose
        if (aprilTagInputs[instanceIndex].timestamps.length == 0) {
          Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
          Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
        }

        // If no recent frames from instance, clear tag poses
        if (Timer.getTimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
          Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
        }
      }

      // Clear demo tag pose
      if (Timer.getTimestamp() - lastDemoTagPoseTimestamp > demoTagPosePersistenceSecs) {
        demoTagPose = null;
      }

      // Log robot poses
      Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
      Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

      // Log tag poses
      List<Pose3d> allTagPoses = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (Timer.getTimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
          aprilTagLayoutSupplier
              .get()
              .getLayout()
              .getTagPose(detectionEntry.getKey())
              .ifPresent(allTagPoses::add);
        }
      }
      Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

      // Log demo tag pose
      if (demoTagPose == null) {
        Logger.recordOutput("AprilTagVision/DemoTagPose", new Pose3d[] {});
      } else {
        Logger.recordOutput("AprilTagVision/DemoTagPose", demoTagPose);
      }
      Logger.recordOutput("AprilTagVision/DemoTagPoseId", new long[] {29});

      // Send results to robot state
      allVisionObservations.stream()
          .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
          .forEach(RobotState.getInstance()::addVisionObservation);
      allTxTyObservations.values().stream().forEach(RobotState.getInstance()::addTxTyObservation);
      allAlgaeTxTyObservations.stream()
          .sorted(Comparator.comparingDouble(AlgaeTxTyObservation::timestamp))
          .forEach(RobotState.getInstance()::addAlgaeTxTyObservation);
    }
  }
}
