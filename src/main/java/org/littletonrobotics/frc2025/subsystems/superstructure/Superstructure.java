// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.util.*;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState.State;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam.Goal;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final Dispenser dispenser;
  private final Slam slam;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  private SuperstructureState state = State.START.getValue();
  private SuperstructureState next = null;
  private SuperstructureState goal = State.START.getValue();

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");
  private final SuperstructureVisualizer goalVisualizer = new SuperstructureVisualizer("Goal");

  public Superstructure(Elevator elevator, Dispenser dispenser, Slam slam) {
    this.elevator = elevator;
    this.dispenser = dispenser;
    this.slam = slam;

    // Add states as vertices
    for (var state : State.values()) {
      graph.addVertex(state.getValue());
    }

    // Populate edges
    // Add edge from start to stow
    graph.addEdge(
        State.START.getValue(),
        State.STOW.getValue(),
        EdgeCommand.builder()
            .command(
                elevator
                    .homingSequence()
                    .deadlineFor(
                        Commands.runOnce(
                            () -> {
                              slam.setGoal(Slam.Goal.IDLE);
                              dispenser.setTunnelVolts(0.0);
                              dispenser.setGripperCurrent(0.0);
                            }))
                    .andThen(
                        runSuperstructurePose(SuperstructurePose.Preset.STOW.getPose()),
                        Commands.waitUntil(this::isAtGoal),
                        runSuperstructureExtras(SuperstructureState.State.STOW.getValue())))
            .build());

    final Set<SuperstructureState> freeNoAlgaeStates =
        Set.of(
            State.STOW.getValue(),
            State.INTAKE.getValue(),
            State.L1_CORAL.getValue(),
            State.L2_CORAL.getValue(),
            State.L3_CORAL.getValue(),
            State.L4_CORAL.getValue(),
            State.ALGAE_FLOOR_INTAKE.getValue(),
            State.ALGAE_L2_INTAKE.getValue(),
            State.ALGAE_L3_INTAKE.getValue());

    final Set<SuperstructureState> freeAlgaeStates =
        Set.of(
            State.ALGAE_STOW_FRONT.getValue(),
            State.L3_CORAL_UNREVERSED.getValue(),
            State.L4_CORAL_UNREVERSED.getValue(),
            State.ALGAE_L2_INTAKE.getValue(),
            State.ALGAE_L3_INTAKE.getValue());

    final Set<SuperstructureState> algaeIntakeStates =
        Set.of(
            State.ALGAE_FLOOR_INTAKE.getValue(),
            State.ALGAE_L2_INTAKE.getValue(),
            State.ALGAE_L3_INTAKE.getValue());

    // Add all free edges
    for (var from : freeNoAlgaeStates) {
      for (var to : freeNoAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from)) {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
        } else {
          graph.addEdge(from, to, getEdgeCommand(from, to));
        }
      }
    }

    for (var from : freeAlgaeStates) {
      for (var to : freeAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from)) {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.ALGAE).build());
        } else {
          graph.addEdge(from, to, getEdgeCommand(from, to));
        }
      }
    }

    for (var from : algaeIntakeStates) {
      for (var to : algaeIntakeStates) {
        if (from == to) continue;
        graph.addEdge(from, to, getEdgeCommand(from, to));
      }
    }

    QuintConsumer<SuperstructureState, SuperstructureState, Boolean, AlgaeEdge, Boolean> addEdge =
        (from, to, restricted, algaeEdgeType, reverse) -> {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder()
                  .restricted(restricted)
                  .algaeEdgeType(algaeEdgeType)
                  .build());
          if (reverse) {
            graph.addEdge(
                to,
                from,
                getEdgeCommand(to, from).toBuilder()
                    .restricted(restricted)
                    .algaeEdgeType(algaeEdgeType)
                    .build());
          }
        };

    // Add edges for paired states
    final Set<Pair<SuperstructureState, SuperstructureState>> pairedStates =
        Set.of(
            new Pair<>(State.L1_CORAL.getValue(), State.L1_CORAL_EJECT.getValue()),
            new Pair<>(State.L2_CORAL.getValue(), State.L2_CORAL_EJECT.getValue()),
            new Pair<>(State.L3_CORAL.getValue(), State.L3_CORAL_EJECT.getValue()),
            new Pair<>(State.L4_CORAL.getValue(), State.L4_CORAL_EJECT.getValue()),
            new Pair<>(
                State.L3_CORAL_REVERSED.getValue(), State.L3_CORAL_REVERSED_EJECT.getValue()),
            new Pair<>(
                State.L4_CORAL_REVERSED.getValue(), State.L4_CORAL_REVERSED_EJECT.getValue()),
            new Pair<>(State.L3_CORAL_REVERSED.getValue(), State.L3_CORAL_UNREVERSED.getValue()),
            new Pair<>(State.L4_CORAL_REVERSED.getValue(), State.L4_CORAL_UNREVERSED.getValue()),
            new Pair<>(State.L3_CORAL_REVERSED.getValue(), State.L4_CORAL_REVERSED.getValue()),
            new Pair<>(State.ALGAE_STOW.getValue(), State.ALGAE_STOW_FRONT.getValue()),
            new Pair<>(State.ALGAE_STOW_FRONT.getValue(), State.PRE_PROCESSOR.getValue()));
    for (var ejectPair : pairedStates) {
      addEdge.apply(ejectPair.getFirst(), ejectPair.getSecond(), false, AlgaeEdge.NONE, true);
    }

    // Add recoverable algae states
    for (var from :
        Set.of(
            State.ALGAE_STOW.getValue(),
            State.PRE_PROCESSOR.getValue(),
            State.ALGAE_STOW_FRONT.getValue(),
            State.L3_CORAL_UNREVERSED.getValue(),
            State.L4_CORAL_UNREVERSED.getValue(),
            State.L3_CORAL_REVERSED.getValue(),
            State.L4_CORAL_REVERSED.getValue(),
            State.L3_CORAL_REVERSED_EJECT.getValue(),
            State.L4_CORAL_REVERSED_EJECT.getValue())) {
      for (var to : freeNoAlgaeStates) {
        graph.addEdge(
            from,
            to,
            getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
      }
    }

    for (var from :
        Set.of(State.PROCESSING.getValue(), State.THROWN.getValue(), State.TOSS.getValue())) {
      for (var to : freeNoAlgaeStates) {
        graph.addEdge(
            from,
            to,
            getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
      }
    }

    // Add miscellaneous edges
    addEdge.apply(
        State.PRE_PROCESSOR.getValue(), State.PROCESSING.getValue(), true, AlgaeEdge.NONE, false);
    addEdge.apply(
        State.ALGAE_STOW_FRONT.getValue(), State.THROWN.getValue(), true, AlgaeEdge.NONE, false);
    addEdge.apply(
        State.ALGAE_STOW_FRONT.getValue(), State.TOSS.getValue(), true, AlgaeEdge.NONE, false);
    addEdge.apply(
        State.PROCESSING.getValue(), State.PRE_PROCESSOR.getValue(), false, AlgaeEdge.ALGAE, false);
    addEdge.apply(
        State.THROWN.getValue(), State.ALGAE_STOW_FRONT.getValue(), false, AlgaeEdge.ALGAE, false);
    addEdge.apply(
        State.TOSS.getValue(), State.ALGAE_STOW_FRONT.getValue(), false, AlgaeEdge.ALGAE, false);
    addEdge.apply(
        State.STOW.getValue(), State.ALGAE_STOW.getValue(), false, AlgaeEdge.ALGAE, false);
    addEdge.apply(
        State.ALGAE_STOW.getValue(), State.STOW.getValue(), false, AlgaeEdge.NO_ALGAE, false);

    setDefaultCommand(
        run(
            () ->
                setGoal(
                    dispenser.hasAlgae() ? State.ALGAE_STOW.getValue() : State.STOW.getValue())));
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();
    dispenser.periodic();
    slam.periodic();

    if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (next != null) {
        state = next;
        next = null;
      }

      // Schedule next command in sequence
      if (state != goal) {
        bfs(state, goal)
            .ifPresent(
                next -> {
                  this.next = next;
                  edgeCommand = graph.getEdge(state, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }

    // Log state
    Logger.recordOutput(
        "Superstructure/State", State.getPreset(state).map(State::toString).orElse(""));
    Logger.recordOutput(
        "Superstructure/Next", State.getPreset(next).map(State::toString).orElse(""));
    Logger.recordOutput(
        "Superstructure/Goal", State.getPreset(goal).map(State::toString).orElse(""));
    if (edgeCommand != null) {
      Logger.recordOutput(
          "Superstructure/EdgeCommand",
          State.getPreset(graph.getEdgeSource(edgeCommand)).map(State::toString).orElse("")
              + " --> "
              + State.getPreset(graph.getEdgeTarget(edgeCommand)).map(State::toString).orElse(""));
    } else {
      Logger.recordOutput("Superstructure/EdgeCommand", "");
    }

    // Update visualizer
    measuredVisualizer.update(
        elevator.getPositionMeters(),
        dispenser.getFinalAngle(),
        slam.isSlammed(),
        slam.isRetracting(),
        dispenser.hasAlgae());
    setpointVisualizer.update(
        elevator.getSetpoint().position,
        Rotation2d.fromRadians(dispenser.getSetpoint().position),
        slam.isSlammed(),
        slam.isRetracting(),
        dispenser.hasAlgae());
    goalVisualizer.update(
        elevator.getGoalMeters(),
        Rotation2d.fromRadians(dispenser.getGoal()),
        true,
        slam.getGoal().isRetracted(),
        dispenser.hasAlgae());
  }

  private void setGoal(SuperstructureState goal) {
    // Don't do anything if goal is the same
    if (this.goal == goal) return;
    this.goal = goal;

    if (next == null) return;

    var edgeToCurrentState = graph.getEdge(next, state);
    // Figure out if we should schedule a different command to get to goal faster
    if (edgeCommand.getCommand().isScheduled()
        && edgeToCurrentState != null
        && isEdgeAllowed(edgeToCurrentState, goal)) {
      // Figure out where we would have gone from the previous state
      bfs(state, goal)
          .ifPresent(
              newNext -> {
                if (newNext == next) {
                  // We are already on track
                  return;
                }

                if (newNext != state && graph.getEdge(next, newNext) != null) {
                  // We can skip directly to the newNext edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(state, newNext);
                  edgeCommand.getCommand().schedule();
                  next = newNext;
                } else {
                  // Follow the reverse edge from next back to the current edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(next, state);
                  edgeCommand.getCommand().schedule();
                  var temp = state;
                  state = next;
                  next = temp;
                }
              });
    }
  }

  public Command runGoal(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
  }

  private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
    System.out.println(
        "bfs: Start "
            + State.getPreset(start).toString()
            + " to Goal "
            + State.getPreset(goal).toString());
    // Map to track the parent of each visited node
    Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
    Queue<SuperstructureState> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureState current = queue.poll();
      // Check if we've reached the goal
      if (current.equals(goal)) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureState neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    // Reconstruct the path to the goal if found
    if (!parents.containsKey(goal)) {
      return Optional.empty(); // Goal not reachable
    }

    // Trace back the path from goal to start
    SuperstructureState nextState = goal;
    while (!nextState.equals(start)) {
      SuperstructureState parent = parents.get(nextState);
      if (parent == null) {
        return Optional.empty(); // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return Optional.of(nextState);
      }
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  /**
   * Run superstructure to {@link SuperstructureState} to while avoiding intake. Ends when all
   * subsystems are complete with profiles.
   */
  private EdgeCommand getEdgeCommand(SuperstructureState from, SuperstructureState to) {
    if ((from == State.ALGAE_STOW_FRONT.getValue() && to == State.PRE_PROCESSOR.getValue())
        || (from == State.PRE_PROCESSOR.getValue() && to == State.ALGAE_STOW_FRONT.getValue())) {
      // Algae Stow Front <--> Pre-Processor
      final boolean toProcessor = to == State.PRE_PROCESSOR.getValue();
      return EdgeCommand.builder()
          .command(
              runSuperstructurePose(to.getPose())
                  .andThen(
                      getSlamCommand(toProcessor ? Goal.SLAM_UP : Goal.SLAM_DOWN),
                      Commands.runOnce(
                          () ->
                              slam.setIntakeVolts(
                                  (toProcessor ? 1.0 : -1.0) * Slam.occupiedVolts.get())),
                      Commands.waitUntil(
                          () ->
                              isAtGoal()
                                  && slam.isSlammed()
                                  && (toProcessor == slam.isRetracting())))
                  .finallyDo(
                      () -> {
                        slam.setGoal(to.getSlamGoal());
                        slam.setIntakeVolts(to.getIntakeVolts().getAsDouble());
                      }))
          .build();

    } else if (from.getHeight() != to.getHeight()) { // Account for different heights and reversing
      boolean shouldSlam =
          from.getHeight() == SuperstructureState.Height.BOTTOM
              || to.getHeight() == SuperstructureState.Height.BOTTOM
              || from.getHeight() == SuperstructureState.Height.INTAKE
              || to.getHeight() == SuperstructureState.Height.INTAKE;
      boolean shouldPivotFirst = from.isReversed() || to.isReversed();
      if (shouldPivotFirst) {
        // Sequence with pivot movement
        return EdgeCommand.builder()
            .command(
                Commands.sequence(
                    runSuperstructurePose(
                        new SuperstructurePose(
                            elevator::getPositionMeters,
                            from.isReversed()
                                ? to.getPose().pivotAngle()
                                : from.getPose().pivotAngle())),
                    Commands.waitUntil(this::isAtGoal),
                    runSuperstructurePose(to.getPose())
                        .andThen(
                            shouldSlam
                                ? getSlamCommand(Goal.SLAM_DOWN)
                                : getSlamCommand(to.getSlamGoal()),
                            Commands.waitUntil(this::isAtGoal),
                            runSuperstructureExtras(to))))
            .build();
      } else {
        // Only sequence with slam, ignore pivot
        return EdgeCommand.builder()
            .command(
                runSuperstructurePose(to.getPose())
                    .andThen(
                        shouldSlam
                            ? getSlamCommand(Goal.SLAM_DOWN)
                            : getSlamCommand(to.getSlamGoal()),
                        Commands.waitUntil(this::isAtGoal),
                        runSuperstructureExtras(to)))
            .build();
      }
    } else {
      // Just run to next state if no restrictions
      return EdgeCommand.builder()
          .command(
              runSuperstructurePose(to.getPose())
                  .andThen(runSuperstructureExtras(to), Commands.waitUntil(this::isAtGoal)))
          .build();
    }
  }

  /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
  private Command runSuperstructurePose(SuperstructurePose pose) {
    return Commands.runOnce(
        () -> {
          elevator.setGoal(pose.elevatorHeight());
          dispenser.setGoal(pose.pivotAngle());
        });
  }

  /** Runs dispenser and slam based on {@link SuperstructureState} state. Ends immediately. */
  private Command runSuperstructureExtras(SuperstructureState state) {
    return Commands.runOnce(
        () -> {
          dispenser.setTunnelVolts(state.getTunnelVolts().getAsDouble());
          dispenser.setGripperCurrent(state.getGripperCurrent().getAsDouble());
          slam.setIntakeVolts(state.getIntakeVolts().getAsDouble());
          slam.setGoal(state.getSlamGoal());
        });
  }

  private Command getSlamCommand(Slam.Goal goal) {
    return Commands.runOnce(() -> slam.setGoal(goal));
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
    return (!edge.isRestricted() || goal.equals(graph.getEdgeTarget(edge)))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || dispenser.hasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  private boolean isAtGoal() {
    return elevator.isAtGoal() && dispenser.isAtGoal();
  }

  /** All edge commands should finish and exit properly. */
  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  @FunctionalInterface
  private interface QuintConsumer<A, B, C, D, E> {
    void apply(A a, B b, C c, D d, E e);
  }
}
