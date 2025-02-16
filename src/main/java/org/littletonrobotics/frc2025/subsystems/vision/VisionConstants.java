// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import lombok.Builder;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.015;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double objDetectConfidenceThreshold = 0.8;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));

  private static int monoExposure = 10000;
  private static int colorExposure = 10000;
  private static double monoGain = 0.3;
  private static double colorGain = 0.3;

  public static LoggedTunableNumber[] pitchAdjustments =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
            };
        case COMPBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust3", 0.0)
            };
        default -> new LoggedTunableNumber[] {};
      };
  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case DEVBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              0.2286 - Units.inchesToMeters(1.0),
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(-20.0))))
                  .id("40265450")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              0.2794,
                              -0.2286 + Units.inchesToMeters(1.0),
                              0.21113,
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(20.0))))
                  .id("40265453")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build()
            };
        case COMPBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(8.875),
                              Units.inchesToMeters(10.5),
                              Units.inchesToMeters(8.25),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-28.125 + pitchAdjustments[0].get()),
                                  Units.degreesToRadians(30.0))))
                  .id("40265450")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(3.25),
                              Units.inchesToMeters(5.0),
                              Units.inchesToMeters(6.4),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-16.875 + pitchAdjustments[1].get()),
                                  Units.degreesToRadians(-4.709))))
                  .id("40270688")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(8.875),
                              Units.inchesToMeters(-10.5),
                              Units.inchesToMeters(8.25),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-28.125 + pitchAdjustments[2].get()),
                                  Units.degreesToRadians(-30.0))))
                  .id("40270704")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .stdDevFactor(1.0)
                  .build(),
              CameraConfig.builder()
                  .pose(
                      () ->
                          new Pose3d(
                              Units.inchesToMeters(-16.0),
                              Units.inchesToMeters(-12.0),
                              Units.inchesToMeters(8.5),
                              new Rotation3d(
                                  0.0,
                                  Units.degreesToRadians(-33.75 + pitchAdjustments[3].get()),
                                  Units.degreesToRadians(176.386))))
                  .id("24737133")
                  .width(1280)
                  .height(720)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(0.8)
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Supplier<Pose3d> pose,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double stdDevFactor) {}

  private VisionConstants() {}
}
