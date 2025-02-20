// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public class SuperstructureConstants {
  public static final double pivotToTunnelFront = 0.177800;
  public static final double pivotToTunnelBack = 0.076200;
  public static final double G = 9.807;
  // From inside face to inside face, measured from CAD
  public static final double firstStageHeight = Units.inchesToMeters(33.0);
  public static final double stageHeight = Units.inchesToMeters(29.00);
  public static final double stageThickness = Units.inchesToMeters(1.0);
  public static final double dispenserToTop = Units.inchesToMeters(5.524938);
  public static final double bottomToDispenser = 0.164326;
  public static final double stageToStage = Units.inchesToMeters(4.144181);

  // 2d position of both superstructure and dispenser origin on robot (x forward from center, y off
  // the ground)
  public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(82.0);
  public static final Translation2d superstructureOrigin2d = new Translation2d(0.0825, 0.029);
  public static final Translation3d superstructureOrigin3d =
      new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());
  public static final Translation2d dispenserOrigin2d =
      superstructureOrigin2d.plus(
          new Translation2d(bottomToDispenser + stageThickness * 2, elevatorAngle));
  public static final Translation3d dispenserOrigin3d =
      new Translation3d(dispenserOrigin2d.getX(), 0.0, dispenserOrigin2d.getY());

  // Height from superstructure origin to bottom face of first stage at maxed height
  // minus the difference from superstructure origin to dispenser origin and from the topped out
  // position to the dispenser
  public static final double elevatorMaxTravel =
      firstStageHeight
          + 2 * (stageHeight - stageToStage)
          + 4 * (stageThickness)
          - dispenserToTop
          - bottomToDispenser;

  public static final double algaeMinPassThroughHeight =
      stageHeight
          - bottomToDispenser
          - dispenserToTop
          + stageHeight
          - (stageToStage - stageThickness);

  public static final double stage1ToStage2Height = 0.80;
  public static final double stage2ToStage3Height = 1.37;

  public static final Rotation2d pivotSafeAngle = Rotation2d.fromDegrees(-60.0);
  public static final LoggedTunableNumber throwHeight =
      new LoggedTunableNumber("Superstructure/Throw/Height", elevatorMaxTravel);
  public static final LoggedTunableNumber throwVelocity =
      new LoggedTunableNumber("Superstructure/Throw/Velocity", 3.0);

  public static final double chariotMaxExtension = 0.442630;
}
