// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData.Height;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam;

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
          .slamGoal(Slam.Goal.SLAM_UP)
          .height(Height.BOTTOM)
          .build()),
  ALGAE_STOW_FRONT(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_STOW_FRONT.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .slamGoal(Slam.Goal.SLAM_DOWN)
          .height(Height.INTAKE)
          .build()),
  TOSS(
      ALGAE_STOW_FRONT.getValue().toBuilder()
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .build()),
  PRE_PROCESSOR(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.PRE_PROCESSOR.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .build()),
  PROCESSING(
      PRE_PROCESSOR.getValue().toBuilder()
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .intakeVolts(Slam.processorVolts)
          .build()),
  L1_CORAL_ALGAE(
      L1_CORAL.getValue().toBuilder().gripperCurrent(Dispenser.gripperIntakeCurrent).build()),
  L1_CORAL_ALGAE_EJECT(
      L1_CORAL_EJECT.getValue().toBuilder().gripperCurrent(Dispenser.gripperIntakeCurrent).build()),
  L3_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L3_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  L4_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L4_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  L3_CORAL_UNREVERSED(
      L3_CORAL_REVERSED.getValue().toBuilder()
          .pose(SuperstructurePose.Preset.L3_CORAL_UNREVERSED.getPose())
          .reversed(false)
          .build()),
  L4_CORAL_UNREVERSED(
      L4_CORAL_REVERSED.getValue().toBuilder()
          .pose(SuperstructurePose.Preset.L4_CORAL_UNREVERSED.getPose())
          .reversed(false)
          .build()),
  L3_CORAL_REVERSED_EJECT(
      L3_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L4_CORAL_REVERSED_EJECT(
      L4_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build());

  private final SuperstructureStateData value;
}
