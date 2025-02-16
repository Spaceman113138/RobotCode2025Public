// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.objectivetracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AutoScore;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class ObjectiveTracker extends VirtualSubsystem {
  private static final String inactiveColor = "#242424";
  private static final String completeColor = "#00ff00";
  private static final String incompleteColor = "#ff0000";
  private static final String skipColor = "#ffff00";

  private static final Set<CoralObjective> l1CoralObjectives =
      IntStream.rangeClosed(0, 11)
          .mapToObj(id -> new CoralObjective(id, ReefLevel.L1))
          .collect(Collectors.toSet());

  private static final LoggedTunableNumber lookaheadS =
      new LoggedTunableNumber("ObjectiveTracker/LookaheadS", 0.15);

  private final ReefControlsIO io;
  private final ReefControlsIOInputsAutoLogged inputs = new ReefControlsIOInputsAutoLogged();
  private int previousLevel1State = 0;
  private final int[] previousLevelStates = new int[] {0, 0, 0};
  private int previousAlgaeState = 63;

  private boolean firstCycle = true;
  private ReefState reefState;
  private ReefState previousReefState;
  private boolean coopState = false;
  private boolean lastCoopState = false;
  private final Set<CoralObjective> availableBranches = new HashSet<>();
  private final Set<CoralObjective> availableUnblockedBranches = new HashSet<>();
  private final Set<AlgaeObjective> presentAlgae = new HashSet<>();
  @Getter private Optional<AlgaeObjective> algaeObjective;

  private List<CoralPriority> fullStrategy = new ArrayList<>();
  private final List<CoralPriority> uncompletedPriorities = new ArrayList<>();
  private List<CoralPriority> openPriorities = new ArrayList<>();

  private final LoggedNetworkString strategyInput =
      new LoggedNetworkString("/SmartDashboard/Strategy", "545352514321");
  private String previousStrategy = "";
  private final StringPublisher[] strategyNamesPublishers = new StringPublisher[8];
  private final StringPublisher[] strategyCompletedPublishers = new StringPublisher[8];
  private final LoggedDashboardChooser<String> dashboardLevelChooser =
      new LoggedDashboardChooser<>("Reef Level");

  @AutoLogOutput(key = "ObjectiveTracker/PredictedPose")
  private Pose2d predictedRobot = new Pose2d();

  public ObjectiveTracker(ReefControlsIO io) {
    this.io = io;

    dashboardLevelChooser.addDefaultOption("Auto", "Auto");
    for (var level : ReefLevel.values()) {
      dashboardLevelChooser.addOption(level.toString(), level.toString());
    }

    var table =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("Objective Tracker");
    for (int i = 0; i < 8; i++) {
      strategyNamesPublishers[i] = table.getStringTopic("Priority #" + (i + 1)).publish();
      strategyCompletedPublishers[i] =
          table.getStringTopic("Priority #" + (i + 1) + " Completed").publish();

      strategyNamesPublishers[i].set("");
      strategyCompletedPublishers[i].set(inactiveColor);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReefControls", inputs);

    // Update strategy when input changed
    boolean strategyChanged = false;
    if (!strategyInput.get().equals(previousStrategy) || firstCycle) {
      strategyChanged = true;
      parseStrategy();
    }

    final int[] inputLevelStates =
        new int[] {inputs.level2State, inputs.level3State, inputs.level4State};
    if (firstCycle) {
      previousLevel1State = inputs.level1State;
      previousLevelStates[0] = inputs.level2State;
      previousLevelStates[1] = inputs.level3State;
      previousLevelStates[2] = inputs.level4State;
      previousAlgaeState = inputs.algaeState;
      coopState = inputs.coopState;

      // Serialize state into reef state object
      boolean[][] coralState = new boolean[3][12];
      for (int level = 0; level < 3; level++) {
        for (int i = 0; i < 12; i++) {
          coralState[level][i] = (inputLevelStates[level] & (1 << i)) != 0;
        }
      }
      boolean[] algaeState = new boolean[6];
      for (int i = 0; i < 6; i++) {
        algaeState[i] = (inputs.algaeState & (1 << i)) != 0;
      }
      reefState = new ReefState(coralState, algaeState, inputs.level1State);
    }

    if (inputs.level1State != previousLevel1State && !firstCycle) {
      reefState = new ReefState(reefState.coral(), reefState.algae(), inputs.level1State);
      previousLevel1State = inputs.level1State;
    }
    for (int i = 0; i < 3; i++) {
      if (inputLevelStates[i] != previousLevelStates[i] && !firstCycle) {
        previousLevelStates[i] = inputLevelStates[i];
        boolean[] levelState = new boolean[12];
        for (int j = 0; j < 12; j++) {
          levelState[j] = (inputLevelStates[i] & (1 << j)) != 0;
        }
        reefState.coral()[i] = levelState;
      }
    }
    if (inputs.algaeState != previousAlgaeState && !firstCycle) {
      previousAlgaeState = inputs.algaeState;
      boolean[] algae = new boolean[6];
      for (int j = 0; j < 6; j++) {
        algae[j] = (inputs.algaeState & (1 << j)) != 0;
      }
      reefState = new ReefState(reefState.coral(), algae, reefState.troughCount());
    }
    if (inputs.coopState != coopState && !firstCycle) {
      coopState = inputs.coopState;
    }

    // If state has changed, recalculate
    if (!reefState.equals(previousReefState)
        || coopState != lastCoopState
        || strategyChanged
        || firstCycle) {
      previousReefState = reefState.clone();
      lastCoopState = coopState;

      // Find available branches
      availableBranches.clear();
      availableUnblockedBranches.clear();
      availableBranches.addAll(l1CoralObjectives);
      availableUnblockedBranches.addAll(l1CoralObjectives);
      for (int level = 0; level < 3; level++) {
        for (int branch = 0; branch < 12; branch++) {
          // Check if coral already placed on branchId
          if (reefState.coral()[level][branch]) continue;
          ReefLevel reefLevel = ReefLevel.valueOf("L" + (level + 2));
          availableBranches.add(new CoralObjective(branch, reefLevel));

          if (level != 2) {
            // For L2 and L3 check algae
            // Get associated algae location
            int face = Math.floorDiv(branch, 2);
            if (level == 1 && reefState.algae()[face]) continue;
            else if (face % 2 == 1 && reefState.algae()[face]) continue;
          }
          // Add to available branches
          availableUnblockedBranches.add(new CoralObjective(branch, reefLevel));
        }
      }
      logAvailableBranches(availableBranches, "AvailableBranches");
      logAvailableBranches(availableUnblockedBranches, "AvailableUnblockedBranches");

      presentAlgae.clear();
      for (int i = 0; i < 6; i++) {
        if (reefState.algae[i]) presentAlgae.add(new AlgaeObjective(i));
      }
      if (presentAlgae.isEmpty()) {
        Logger.recordOutput("ObjectiveTracker/ReefState/PresentAlgae", new AlgaeObjective[] {});
      } else {
        Logger.recordOutput(
            "ObjectiveTracker/ReefState/PresentAlgae", presentAlgae.toArray(AlgaeObjective[]::new));
      }

      // Update uncompletedPriorities
      uncompletedPriorities.clear();
      Optional<CoralPriority> uselessPriority = Optional.empty();
      if (coopState) {
        // Decide which 5 coral we should give up on based on completion
        uselessPriority =
            Set.of(CoralPriority._54, CoralPriority._53, CoralPriority._52, CoralPriority._51)
                .stream()
                .filter(coralPriority -> !coralPriority.complete(reefState))
                .sorted(
                    Comparator.comparingInt(
                            priority -> {
                              int index = fullStrategy.indexOf(priority);
                              return index == -1 ? Integer.MAX_VALUE : index;
                            })
                        .reversed())
                .sorted(
                    Comparator.comparingInt(
                        priority -> {
                          if (priority.getLevel() == ReefLevel.L1) return reefState.troughCount;
                          int count = 0;
                          for (int i = 0; i < 12; i++) {
                            if (reefState.coral[priority.getLevel().ordinal() - 1][i]) count++;
                          }
                          return count;
                        }))
                .findFirst();
      }

      for (int i = 0; i < fullStrategy.size(); i++) {
        // Show dashboard
        if (uselessPriority.isPresent() && uselessPriority.get() == fullStrategy.get(i)) {
          strategyCompletedPublishers[i].set(skipColor);
        } else if (fullStrategy.get(i).complete(reefState)) {
          strategyCompletedPublishers[i].set(completeColor);
        } else {
          strategyCompletedPublishers[i].set(incompleteColor);
          uncompletedPriorities.add(fullStrategy.get(i));
        }
      }

      // Log reef state
      for (int i = 0; i < reefState.coral.length; i++) {
        for (int j = 0; j < reefState.coral[i].length; j++) {
          Logger.recordOutput(
              "ObjectiveTracker/ReefState/CoralState/Level" + (i + 2) + "/Branch" + (j + 1),
              reefState.coral[i][j]);
        }
      }
      for (int i = 0; i < reefState.algae.length; i++) {
        Logger.recordOutput("ObjectiveTracker/ReefState/AlgaeState/" + (i + 1), reefState.algae[i]);
      }

      // Show state of reef in 3d
      Set<Pose3d> coralPoses = new HashSet<>();
      for (int level = 0; level < reefState.coral().length; level++) {
        for (int j = 0; j < reefState.coral()[0].length; j++) {
          if (!reefState.coral()[level][j]) continue;
          coralPoses.add(
              AllianceFlipUtil.apply(
                  FieldConstants.Reef.branchPositions
                      .get(j)
                      .get(ReefLevel.fromLevel(level + 1))
                      .transformBy(
                          GeomUtil.toTransform3d(
                              new Pose3d(
                                  new Translation3d(-Units.inchesToMeters(11.875) / 2.5, 0.0, 0.0),
                                  new Rotation3d())))));
        }
      }
      Logger.recordOutput("ObjectiveTracker/3DView/Coral", coralPoses.toArray(Pose3d[]::new));

      Set<Translation3d> algaePoses = new HashSet<>();
      for (int i = 0; i < 6; i++) {
        if (!reefState.algae()[i]) continue;
        var firstBranchPose = FieldConstants.Reef.branchPositions.get(i * 2).get(ReefLevel.L2);
        var secondBranchPose = FieldConstants.Reef.branchPositions.get(i * 2 + 1).get(ReefLevel.L3);
        algaePoses.add(
            AllianceFlipUtil.apply(
                firstBranchPose
                    .getTranslation()
                    .interpolate(secondBranchPose.getTranslation(), 0.5)
                    .plus(
                        new Translation3d(
                            -FieldConstants.algaeDiameter / 3.0,
                            new Rotation3d(
                                0.0,
                                -35.0 / 180.0 * Math.PI,
                                firstBranchPose.getRotation().getZ())))
                    .plus(
                        new Translation3d(
                            0.0,
                            0.0,
                            (i % 2 == 0)
                                ? secondBranchPose.getZ() - firstBranchPose.getZ()
                                : 0.0))));
      }
      Logger.recordOutput(
          "ObjectiveTracker/3DView/Algae", algaePoses.toArray(Translation3d[]::new));
    }

    // Publish names
    for (int i = 0; i < fullStrategy.size(); i++) {
      strategyNamesPublishers[i].set(fullStrategy.get(i).getName());
    }
    for (int i = fullStrategy.size(); i < 8; i++) {
      strategyNamesPublishers[i].set("");
    }

    // Publish state to dashboard
    io.setSelectedLevel(inputs.selectedLevel);
    io.setLevel1State(reefState.troughCount());
    final int[] levelStates = new int[] {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 12; j++) {
        if (reefState.coral()[i][j]) {
          levelStates[i] |= 1 << j;
        }
      }
    }
    io.setLevel2State(levelStates[0]);
    io.setLevel3State(levelStates[1]);
    io.setLevel4State(levelStates[2]);
    int algaeState = 0;
    for (int i = 0; i < 6; i++) {
      if (reefState.algae()[i]) {
        algaeState |= 1 << i;
      }
    }
    io.setAlgaeState(algaeState);
    io.setCoopState(coopState);

    // Calculate predicted robot
    predictedRobot =
        RobotState.getInstance()
            .getEstimatedPose()
            .exp(RobotState.getInstance().getRobotVelocity().toTwist2d(lookaheadS.get()));
    final Pose2d flippedPredictedPose = AllianceFlipUtil.apply(predictedRobot);

    // Calculate current open priorities in order
    openPriorities =
        uncompletedPriorities.stream()
            .filter(
                priority ->
                    !priority.getAvailableCoral(flippedPredictedPose, availableBranches).isEmpty())
            .toList();
    Logger.recordOutput(
        "ObjectiveTracker/Strategy/OpenPriorities", openPriorities.toArray(CoralPriority[]::new));
    // Show strategy on LEDs
    Leds.getInstance().firstPriorityLevel = getLevel(false);
    Leds.getInstance().secondPriorityLevel = getLevel(true);
    Leds.getInstance().firstPriorityBlocked =
        !openPriorities.isEmpty()
            && openPriorities
                .get(0)
                .getAvailableCoral(flippedPredictedPose, availableUnblockedBranches)
                .isEmpty();
    Leds.getInstance().secondPriorityBlocked =
        openPriorities.size() <= 1
            || openPriorities
                .get(1)
                .getAvailableCoral(flippedPredictedPose, availableUnblockedBranches)
                .isEmpty();

    // Get nearest algae objective
    algaeObjective =
        presentAlgae.stream()
            .filter(
                objective ->
                    flippedPredictedPose.relativeTo(AutoScore.getReefIntakePose(objective)).getX()
                        <= 0.1)
            .min(
                Comparator.comparingDouble(
                    objective ->
                        flippedPredictedPose
                            .getTranslation()
                            .getDistance(AutoScore.getReefIntakePose(objective).getTranslation())));
    algaeObjective.ifPresentOrElse(
        objective ->
            Logger.recordOutput(
                "ObjectiveTracker/Strategy/AlgaeObjective", new AlgaeObjective[] {objective}),
        () ->
            Logger.recordOutput(
                "ObjectiveTracker/Strategy/AlgaeObjective", new AlgaeObjective[] {}));

    // Reset first cycle indicator
    firstCycle = false;
  }

  public Command requestScored(Supplier<CoralObjective> coralObjective) {
    return Commands.runOnce(
        () -> {
          if (coralObjective.get().reefLevel() == ReefLevel.L1) {
            reefState =
                new ReefState(reefState.coral(), reefState.algae(), reefState.troughCount + 1);
          } else {
            reefState
                    .coral()[coralObjective.get().reefLevel().ordinal() - 1][
                    coralObjective.get().branchId()] =
                true;
          }
        });
  }

  public Command requestAlgaeIntaked(Supplier<AlgaeObjective> algaeObjective) {
    return Commands.runOnce(() -> reefState.algae()[algaeObjective.get().id()] = false);
  }

  public Optional<ReefLevel> getLevel(boolean secondPriority) {
    if (!dashboardLevelChooser.get().equals("Auto")) {
      return Optional.of(ReefLevel.valueOf(dashboardLevelChooser.get()));
    }
    var list = openPriorities.stream().map(CoralPriority::getLevel).distinct().toList();
    if (!secondPriority) {
      return list.stream().findFirst();
    } else {
      if (list.size() < 2) {
        return Optional.empty();
      }
      return Optional.of(list.get(1));
    }
  }

  public Optional<CoralObjective> getCoralObjective(ReefLevel level) {
    Pose2d flippedRobot = AllianceFlipUtil.apply(predictedRobot);
    return openPriorities.stream()
        .filter(priority -> priority.getLevel() == level)
        .findFirst()
        .flatMap(
            coralPriority ->
                coralPriority.getAvailableCoral(flippedRobot, availableBranches).stream()
                    .filter(availableUnblockedBranches::contains)
                    .min(nearestCoralObjectiveComparator(flippedRobot)));
  }

  private static final Set<Character> allowedCharacters = Set.of('1', '2', '3', '4', '5');

  private void parseStrategy() {
    previousStrategy = strategyInput.get();
    fullStrategy.clear();
    String filtered =
        strategyInput
            .get()
            .chars()
            .mapToObj(chr -> (char) chr)
            .filter(allowedCharacters::contains)
            .collect(
                Collector.of(
                    StringBuilder::new,
                    StringBuilder::append,
                    StringBuilder::append,
                    StringBuilder::toString));
    Logger.recordOutput("ObjectiveTracker/Strategy/StrategyInput", filtered);
    // Populate with proper enums
    for (int i = 0; i < filtered.length(); i++) {
      if (filtered.charAt(i) == '5' && i + 1 < filtered.length() && filtered.charAt(i + 1) != '5') {
        fullStrategy.add(CoralPriority.valueOf("_" + filtered.substring(i, i + 2)));
        i++;
      } else if (filtered.charAt(i) != '5') {
        fullStrategy.add(CoralPriority.valueOf("_" + filtered.charAt(i)));
      }
    }
    // Remove duplicates
    fullStrategy = new ArrayList<>(fullStrategy.stream().distinct().toList());
    // Add fill priorities
    for (int i = 4; i >= 1; i--) {
      CoralPriority priority = CoralPriority.valueOf("_" + i);
      if (!fullStrategy.contains(priority)) {
        fullStrategy.add(priority);
      }
    }
    Logger.recordOutput(
        "ObjectiveTracker/Strategy/FullStrategy", fullStrategy.toArray(CoralPriority[]::new));

    for (int i = 0; i < fullStrategy.size(); i++) {
      strategyNamesPublishers[i].set(fullStrategy.get(i).getName());
      strategyCompletedPublishers[i].set(inactiveColor);
    }
    for (int i = fullStrategy.size(); i < 8; i++) {
      strategyNamesPublishers[i].set("");
      strategyCompletedPublishers[i].set(inactiveColor);
    }
  }

  @Getter
  @RequiredArgsConstructor
  /** All strategies have bias towards higher levels! */
  public enum CoralPriority {
    _54(ReefLevel.L4, false, "5/4"),
    _53(ReefLevel.L3, false, "5/3"),
    _52(ReefLevel.L2, false, "5/2"),
    _51(ReefLevel.L1, false, "5/1"),
    _4(ReefLevel.L4, true, "4"),
    _3(ReefLevel.L3, true, "3"),
    _2(ReefLevel.L2, true, "2"),
    _1(ReefLevel.L1, true, "1");

    private final ReefLevel level;
    private final boolean fill;
    private final String name;

    public boolean complete(ReefState reefState) {
      if (fill) {
        if (level == ReefLevel.L1) return false;
        boolean[] levelState = reefState.coral()[level.ordinal() - 1];
        for (boolean b : levelState) {
          if (!b) return false;
        }
        return true;
      }
      if (level == ReefLevel.L1) return reefState.troughCount() >= 5;
      int count = 0;
      boolean[] levelState = reefState.coral()[level.ordinal() - 1];
      for (boolean b : levelState) {
        if (b) count++;
      }
      return count >= 5;
    }

    public Set<CoralObjective> getAvailableCoral(
        Pose2d flippedRobot, Set<CoralObjective> availableBranches) {
      return availableBranches.stream()
          .filter(
              objective ->
                  Math.abs(
                              flippedRobot
                                  .relativeTo(AutoScore.getBranchPose(objective))
                                  .getTranslation()
                                  .getAngle()
                                  .getDegrees())
                          <= 100.0
                      && objective.reefLevel() == level) // In front of branch
          .collect(Collectors.toSet());
    }
  }

  private static void logAvailableBranches(Set<CoralObjective> availableBranches, String key) {
    var branchesForLevel =
        availableBranches.stream().collect(Collectors.groupingBy(CoralObjective::reefLevel));
    branchesForLevel.forEach(
        (level, objectives) ->
            Logger.recordOutput(
                "ObjectiveTracker/ReefState/" + key + "/Level" + (level.ordinal() + 1),
                objectives.toArray(CoralObjective[]::new)));
    for (var level : ReefLevel.values()) {
      if (branchesForLevel.get(level) != null) continue;
      Logger.recordOutput(
          "ObjectiveTracker/ReefState/" + key + "/Level" + (level.ordinal() + 1),
          new CoralObjective[] {});
    }
  }

  private static Comparator<CoralObjective> nearestCoralObjectiveComparator(Pose2d robot) {
    return Comparator.comparingDouble(
        (CoralObjective coralObjective) ->
            robot
                .getTranslation()
                .getDistance(AutoScore.getCoralScorePose(coralObjective, false).getTranslation()));
  }

  public record ReefState(boolean[][] coral, boolean[] algae, int troughCount) {
    @Override
    public boolean equals(Object o) {
      if (!(o instanceof ReefState reefState)) return false;
      return troughCount == reefState.troughCount
          && Arrays.equals(algae, reefState.algae)
          && Arrays.deepEquals(coral, reefState.coral);
    }

    @Override
    protected ReefState clone() {
      boolean[][] copy = new boolean[coral.length][coral[0].length];
      for (int i = 0; i < copy.length; i++) {
        copy[i] = Arrays.copyOf(coral[i], coral[i].length);
      }
      return new ReefState(copy, Arrays.copyOf(algae, algae.length), troughCount);
    }
  }
}
