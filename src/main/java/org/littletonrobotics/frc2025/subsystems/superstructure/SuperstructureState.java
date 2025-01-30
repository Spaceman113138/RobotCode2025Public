// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam;

@Builder(toBuilder = true, access = AccessLevel.PRIVATE)
@Getter
public class SuperstructureState {
  @Builder.Default private final SuperstructurePose pose = new SuperstructurePose();
  @Builder.Default private final DoubleSupplier tunnelVolts = () -> 0.0;
  @Builder.Default private final DoubleSupplier gripperCurrent = () -> 0.0;
  @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;
  @Builder.Default private final Slam.Goal slamGoal = Slam.Goal.SLAM_UP;
  @Builder.Default private final Height height = Height.BOTTOM;
  @Builder.Default private final boolean reversed = false;

  @Getter
  public enum State {
    START(SuperstructureState.builder().build()),
    STOW(SuperstructureState.builder().pose(SuperstructurePose.Preset.STOW.getPose()).build()),
    INTAKE(SuperstructureState.builder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
    L1_CORAL(SuperstructureState.builder().pose(SuperstructurePose.Preset.L1.getPose()).build()),
    L2_CORAL(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.L2.getPose())
            .height(Height.FIRST_STAGE)
            .build()),
    L3_CORAL(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.L3.getPose())
            .height(Height.FIRST_STAGE)
            .build()),
    L4_CORAL(
        SuperstructureState.builder()
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
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.ALGAE_FLOOR_INTAKE.getPose())
            .slamGoal(Slam.Goal.SLAM_DOWN)
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .height(Height.BOTTOM)
            .build()),
    ALGAE_L2_INTAKE(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.ALGAE_L2_INTAKE.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .height(Height.FIRST_STAGE)
            .build()),
    ALGAE_L3_INTAKE(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.ALGAE_L3_INTAKE.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .height(Height.FIRST_STAGE)
            .build()),
    THROWN(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.THROW.getPose())
            .gripperCurrent(Dispenser.gripperDispenseCurrent)
            .height(Height.SECOND_STAGE)
            .build()),
    ALGAE_STOW(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.ALGAE_STOW.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .slamGoal(Slam.Goal.SLAM_UP)
            .height(Height.BOTTOM)
            .build()),
    ALGAE_STOW_FRONT(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.ALGAE_STOW_FRONT.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .slamGoal(Slam.Goal.SLAM_DOWN)
            .height(Height.INTAKE)
            .build()),
    TOSS(
        State.ALGAE_STOW_FRONT.getValue().toBuilder()
            .gripperCurrent(Dispenser.gripperDispenseCurrent)
            .build()),
    PRE_PROCESSOR(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.PRE_PROCESSOR.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .build()),
    PROCESSING(
        State.PRE_PROCESSOR.getValue().toBuilder()
            .gripperCurrent(Dispenser.gripperDispenseCurrent)
            .intakeVolts(Slam.processorVolts)
            .build()),
    L3_CORAL_REVERSED(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.L3_CORAL_REVERSED.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .height(Height.SECOND_STAGE)
            .build()),
    L4_CORAL_REVERSED(
        SuperstructureState.builder()
            .pose(SuperstructurePose.Preset.L4_CORAL_REVERSED.getPose())
            .gripperCurrent(Dispenser.gripperIntakeCurrent)
            .height(Height.SECOND_STAGE)
            .build()),
    L3_CORAL_UNREVERSED(
        State.L3_CORAL_REVERSED.getValue().toBuilder()
            .pose(SuperstructurePose.Preset.L3_CORAL_UNREVERSED.getPose())
            .build()),
    L4_CORAL_UNREVERSED(
        State.L4_CORAL_REVERSED.getValue().toBuilder()
            .pose(SuperstructurePose.Preset.L4_CORAL_UNREVERSED.getPose())
            .build()),
    L3_CORAL_REVERSED_EJECT(
        L3_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
    L4_CORAL_REVERSED_EJECT(
        L4_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build());

    private final SuperstructureState value;

    State(SuperstructureState value) {
      this.value = value;
    }

    public static Optional<State> getPreset(SuperstructureState value) {
      return Arrays.stream(State.values())
          .filter(state -> state.getValue().equals(value))
          .toList()
          .stream()
          .findFirst();
    }
  }

  /** What height is the carriage above? */
  public enum Height {
    BOTTOM,
    INTAKE,
    FIRST_STAGE,
    SECOND_STAGE
  }
}
