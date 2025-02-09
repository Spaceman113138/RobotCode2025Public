// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData.Height;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam.Goal;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  START(SuperstructureStateData.builder().build()),
  STOW(SuperstructureStateData.builder().pose(SuperstructurePose.Preset.STOW.getPose()).build()),
  INTAKE(SuperstructureStateData.builder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L1_CORAL(SuperstructureStateData.builder().pose(SuperstructurePose.Preset.L1.getPose()).build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L2.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L3.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L4_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L4.getPose())
          .height(Height.SECOND_STAGE)
          .build()),
  L1_CORAL_EJECT(
      L1_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L2_CORAL_EJECT(
      L2_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L3_CORAL_EJECT(
      L3_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L4_CORAL_EJECT(
      L4_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  ALGAE_FLOOR_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_FLOOR_INTAKE.getPose())
          .slamGoal(Slam.Goal.SLAM_DOWN)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .intakeVolts(Slam.floorIntakeVolts)
          .height(Height.BOTTOM)
          .build()),
  ALGAE_L2_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_L2_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L3_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_L3_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  THROWN(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.THROW.getPose())
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  ALGAE_STOW(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_STOW.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .slamGoal(Slam.Goal.SLAM_DOWN)
          .height(Height.INTAKE)
          .build()),
  TOSS(
      SuperstructureStateData.builder()
          .pose(
              new SuperstructurePose(
                  SuperstructurePose.Preset.ALGAE_STOW.getPose().elevatorHeight(),
                  () -> Rotation2d.fromDegrees(30.0)))
          .slamGoal(Goal.SLAM_UP)
          .height(Height.INTAKE)
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .build()),
  PRE_PROCESSOR(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.PRE_PROCESSOR.getPose())
          .slamGoal(Goal.SLAM_UP)
          .intakeVolts(Slam.occupiedVolts)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .build()),
  PROCESSING(
      PRE_PROCESSOR.getValue().toBuilder()
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .intakeVolts(() -> 0.0)
          .build()),
  L1_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L1_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L2_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L2_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L3_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L3_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L4_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L4_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  L1_CORAL_REVERSED_EJECT(
      L1_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L2_CORAL_REVERSED_EJECT(
      L2_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L3_CORAL_REVERSED_EJECT(
      L3_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L4_CORAL_REVERSED_EJECT(
      L4_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  UNREVERSED(
      SuperstructureStateData.builder()
          .pose(new SuperstructurePose(() -> 0.0, () -> SuperstructureConstants.pivotSafeAngle))
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE) // Always above second stage
          .build());

  private final SuperstructureStateData value;
}
