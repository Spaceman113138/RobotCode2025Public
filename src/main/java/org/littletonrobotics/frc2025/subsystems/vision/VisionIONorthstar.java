// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2025.FieldConstants;

public class VisionIONorthstar implements VisionIO {
  private static final String[] cameraIds =
      new String[] {"/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0"};

  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 10;
  private static final double cameraGain = 0.3;

  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber demoObservationSubscriber;
  private final DoubleArraySubscriber objDetectObservationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final IntegerSubscriber fpsObjDetectSubscriber;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();

  public VisionIONorthstar(int index) {
    var northstarTable = NetworkTableInstance.getDefault().getTable("northstar_" + index);
    var configTable = northstarTable.getSubTable("config");
    configTable.getStringTopic("camera_id").publish().set(cameraIds[index]);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    configTable.getDoubleTopic("camera_gain").publish().set(cameraGain);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
    isRecordingPublisher.set(false);
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    try {
      configTable
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(Vision.fieldLayout));
    } catch (JsonProcessingException e) {
      throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    demoObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("demo_observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    objDetectObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
    fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

    disconnectedAlert =
        new Alert("No data from \"northstar_" + index + "\"", Alert.AlertType.kError);
    disconnectedTimer.start();
  }

  public void updateInputs(
      AprilTagVisionIOInputs aprilTagInputs, ObjDetectVisionIOInputs objDetectInputs) {
    // Publish timestamp
    if (RobotController.isSystemTimeValid()) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
    }

    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    aprilTagInputs.timestamps = new double[aprilTagQueue.length];
    aprilTagInputs.frames = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      aprilTagInputs.frames[i] = aprilTagQueue[i].value;
    }
    aprilTagInputs.demoFrame = new double[] {};
    for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
      aprilTagInputs.demoFrame = demoFrame;
    }
    aprilTagInputs.fps = fpsAprilTagsSubscriber.get();

    // Get object detection data
    var objDetectQueue = objDetectObservationSubscriber.readQueue();
    objDetectInputs.timestamps = new double[objDetectQueue.length];
    objDetectInputs.frames = new double[objDetectQueue.length][];
    for (int i = 0; i < objDetectQueue.length; i++) {
      objDetectInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
      objDetectInputs.frames[i] = objDetectQueue[i].value;
    }
    objDetectInputs.fps = fpsObjDetectSubscriber.get();

    // Update disconnected alert
    if (aprilTagQueue.length > 0 || objDetectQueue.length > 0) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }

  public void setRecording(boolean active) {
    isRecordingPublisher.set(active);
  }
}
