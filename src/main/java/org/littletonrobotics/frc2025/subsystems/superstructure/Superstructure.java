// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.Slam.Goal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final Dispenser dispenser;
  private final Slam slam;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  @Getter private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  @Getter private SuperstructureState goal = SuperstructureState.START;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  @Setter private BooleanSupplier disabledOverride = () -> false;
  private final Alert driverDisableAlert =
      new Alert("Superstructure disabled due to driver override.", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
          Alert.AlertType.kError);

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");
  private final SuperstructureVisualizer goalVisualizer = new SuperstructureVisualizer("Goal");

  public Superstructure(Elevator elevator, Dispenser dispenser, Slam slam) {
    this.elevator = elevator;
    this.dispenser = dispenser;
    this.slam = slam;

    // Updating E Stop based on disabled override
    new Trigger(() -> disabledOverride.getAsBoolean())
        .onFalse(Commands.runOnce(() -> isEStopped = false).ignoringDisable(true));

    // Add states as vertices
    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }

    // Populate edges
    // Add edge from start to stow
    graph.addEdge(
        SuperstructureState.START,
        SuperstructureState.STOW,
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
                        runSuperstructureExtras(SuperstructureState.STOW)))
            .build());

    final Set<SuperstructureState> freeNoAlgaeStates =
        Set.of(
            SuperstructureState.STOW,
            SuperstructureState.L1_CORAL,
            SuperstructureState.L2_CORAL,
            SuperstructureState.L3_CORAL,
            SuperstructureState.L4_CORAL,
            SuperstructureState.ALGAE_FLOOR_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE);

    final Set<SuperstructureState> freeAlgaeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.UNREVERSED);

    final Set<SuperstructureState> algaeIntakeStates =
        Set.of(
            SuperstructureState.ALGAE_FLOOR_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE);

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
            new Pair<>(SuperstructureState.STOW, SuperstructureState.INTAKE),
            new Pair<>(SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT),
            new Pair<>(SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT),
            new Pair<>(SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT),
            new Pair<>(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT),
            new Pair<>(
                SuperstructureState.L1_CORAL_REVERSED, SuperstructureState.L1_CORAL_REVERSED_EJECT),
            new Pair<>(
                SuperstructureState.L2_CORAL_REVERSED, SuperstructureState.L2_CORAL_REVERSED_EJECT),
            new Pair<>(
                SuperstructureState.L3_CORAL_REVERSED, SuperstructureState.L3_CORAL_REVERSED_EJECT),
            new Pair<>(
                SuperstructureState.L4_CORAL_REVERSED, SuperstructureState.L4_CORAL_REVERSED_EJECT),
            new Pair<>(SuperstructureState.ALGAE_STOW, SuperstructureState.PRE_PROCESSOR));
    for (var ejectPair : pairedStates) {
      addEdge.accept(ejectPair.getFirst(), ejectPair.getSecond(), false, AlgaeEdge.NONE, true);
    }

    // Add recoverable algae states
    for (var from :
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.PRE_PROCESSOR,
            SuperstructureState.UNREVERSED,
            SuperstructureState.PROCESSING,
            SuperstructureState.THROWN,
            SuperstructureState.TOSS)) {
      for (var to : freeNoAlgaeStates) {
        graph.addEdge(
            from,
            to,
            getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
      }
    }

    // Add edges to reversed states
    final Set<SuperstructureState> reversedStates =
        Set.of(
            SuperstructureState.L1_CORAL_REVERSED,
            SuperstructureState.L2_CORAL_REVERSED,
            SuperstructureState.L3_CORAL_REVERSED,
            SuperstructureState.L4_CORAL_REVERSED);
    for (var reversed : reversedStates) {
      addEdge.accept(reversed, SuperstructureState.UNREVERSED, false, AlgaeEdge.NONE, true);
      for (var reversedOther : reversedStates) {
        if (reversedOther == reversed) continue;
        addEdge.accept(reversed, reversedOther, false, AlgaeEdge.NONE, false);
      }
    }

    // Add miscellaneous edges
    addEdge.accept(
        SuperstructureState.PRE_PROCESSOR,
        SuperstructureState.PROCESSING,
        true,
        AlgaeEdge.NONE,
        false);
    addEdge.accept(
        SuperstructureState.ALGAE_STOW, SuperstructureState.THROWN, true, AlgaeEdge.NONE, false);
    addEdge.accept(
        SuperstructureState.ALGAE_STOW, SuperstructureState.TOSS, true, AlgaeEdge.NONE, false);
    addEdge.accept(
        SuperstructureState.PROCESSING,
        SuperstructureState.PRE_PROCESSOR,
        false,
        AlgaeEdge.ALGAE,
        false);
    addEdge.accept(
        SuperstructureState.THROWN, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge.accept(
        SuperstructureState.TOSS, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge.accept(
        SuperstructureState.STOW, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge.accept(
        SuperstructureState.ALGAE_STOW, SuperstructureState.STOW, false, AlgaeEdge.NO_ALGAE, false);

    setDefaultCommand(
        runGoal(
            () ->
                dispenser.hasAlgae() ? SuperstructureState.ALGAE_STOW : SuperstructureState.STOW));
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();
    dispenser.periodic();
    slam.periodic();

    if (DriverStation.isDisabled()) {
      next = null;
    } else if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
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

    // Tell elevator we are stowed
    elevator.setStowed(state == SuperstructureState.STOW);

    // E Stop Dispenser and Elevator if Necessary
    isEStopped =
        (isEStopped
                || elevator.isShouldEStop()
                || (dispenser.isShouldEStop() && Constants.getRobot() != RobotType.DEVBOT))
            && Constants.getMode() != Mode.SIM;
    elevator.setEStopped(isEStopped);
    dispenser.setEStopped(isEStopped);

    driverDisableAlert.set(disabledOverride.getAsBoolean());
    emergencyDisableAlert.set(isEStopped);
    Leds.getInstance().superstructureEstopped = isEStopped;

    // Log state
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/Next", next);
    Logger.recordOutput("Superstructure/Goal", goal);
    if (edgeCommand != null) {
      Logger.recordOutput(
          "Superstructure/EdgeCommand",
          graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
    } else {
      Logger.recordOutput("Superstructure/EdgeCommand", "");
    }

    // Update visualizer
    measuredVisualizer.update(
        elevator.getPositionMeters(),
        dispenser.getPivotAngle(),
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

  @AutoLogOutput(key = "Superstructure/AtGoal")
  public boolean atGoal() {
    return state == goal;
  }

  public boolean hasAlgae() {
    return dispenser.hasAlgae();
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

  public Command runGoal(Supplier<SuperstructureState> goal) {
    return run(() -> setGoal(goal.get()));
  }

  private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
    System.out.println("bfs: Start " + start.toString() + " to Goal " + goal.toString());
    // Map to track the parent of each visited node
    Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
    Queue<SuperstructureState> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureState current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
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
    if ((from == SuperstructureState.ALGAE_STOW && to == SuperstructureState.THROWN)) {
      // Algae Stow Front --> Thrown
      return EdgeCommand.builder()
          .command(
              Commands.runOnce(
                      () -> {
                        elevator.setGoal(
                            () ->
                                new TrapezoidProfile.State(
                                    SuperstructureConstants.throwHeight.get(),
                                    SuperstructureConstants.throwVelocity.get()));
                        dispenser.setGoal(to.getValue().getPose().pivotAngle());
                      })
                  .andThen(
                      getSlamCommand(Goal.SLAM_DOWN),
                      Commands.waitUntil(this::isAtGoal),
                      runSuperstructurePose(to.getValue().getPose()),
                      runSuperstructureExtras(to),
                      Commands.waitUntil(this::isAtGoal)))
          .build();
    } else if ((from == SuperstructureState.ALGAE_STOW && to == SuperstructureState.PRE_PROCESSOR)
        || (from == SuperstructureState.PRE_PROCESSOR && to == SuperstructureState.ALGAE_STOW)) {
      // Algae Stow Front <--> Pre-Processor
      // Algae Stow Front <--> Pre-Processor
      final boolean toProcessor = to == SuperstructureState.PRE_PROCESSOR;
      return EdgeCommand.builder()
          .command(
              runSuperstructurePose(to.getValue().getPose())
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
                        slam.setGoal(to.getValue().getSlamGoal());
                        slam.setIntakeVolts(to.getValue().getIntakeVolts().getAsDouble());
                      }))
          .build();
    } else if (from == SuperstructureState.UNREVERSED
        && to.getValue().isReversed()) { // Handle reversed states
      return EdgeCommand.builder()
          .command(
              runDispenserPivot(to.getValue().getPose().pivotAngle())
                  .andThen(
                      Commands.waitUntil(this::isAtGoal),
                      runSuperstructurePose(to.getValue().getPose()),
                      Commands.waitUntil(this::isAtGoal),
                      runSuperstructureExtras(to)))
          .build();
    } else if (from.getValue().isReversed() && to == SuperstructureState.UNREVERSED) {
      return EdgeCommand.builder()
          .command(
              runElevator(
                      () ->
                          MathUtil.clamp(
                              elevator.getPositionMeters(),
                              SuperstructureConstants.algaeMinPassThroughHeight,
                              SuperstructureConstants.elevatorMaxTravel))
                  .andThen(
                      Commands.waitUntil(this::isAtGoal),
                      runDispenserPivot(to.getValue().getPose().pivotAngle()),
                      Commands.waitUntil(this::isAtGoal)))
          .build();
    } else if (to == SuperstructureState.UNREVERSED) {
      return EdgeCommand.builder()
          .command(
              addSlamAvoidance(
                      runElevator(
                              () ->
                                  MathUtil.clamp(
                                      elevator.getPositionMeters(),
                                      SuperstructureConstants.algaeMinPassThroughHeight,
                                      SuperstructureConstants.elevatorMaxTravel))
                          .alongWith(runDispenserPivot(to.getValue().getPose().pivotAngle()))
                          .andThen(Commands.waitUntil(this::isAtGoal)),
                      from,
                      to)
                  .andThen(runSuperstructureExtras(to)))
          .build();
    } else {
      // Just run to next state if no restrictions
      return EdgeCommand.builder()
          .command(
              addSlamAvoidance(
                      runSuperstructurePose(to.getValue().getPose())
                          .andThen(Commands.waitUntil(this::isAtGoal)),
                      from,
                      to)
                  .andThen(runSuperstructureExtras(to)))
          .build();
    }
  }

  private Command addSlamAvoidance(
      Command command, SuperstructureState from, SuperstructureState to) {
    return Commands.sequence(
        willSlam(from, to) ? getSlamCommand(Goal.SLAM_DOWN) : Commands.none(),
        command,
        getSlamCommand(to.getValue().getSlamGoal()));
  }

  private Command runElevator(DoubleSupplier elevatorHeight) {
    return Commands.runOnce(() -> elevator.setGoal(elevatorHeight));
  }

  private Command runDispenserPivot(Supplier<Rotation2d> pivotAngle) {
    return Commands.runOnce(() -> dispenser.setGoal(pivotAngle));
  }

  /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
  private Command runSuperstructurePose(SuperstructurePose pose) {
    return runElevator(pose.elevatorHeight()).alongWith(runDispenserPivot(pose.pivotAngle()));
  }

  /** Runs dispenser and slam based on {@link SuperstructureState} state. Ends immediately. */
  private Command runSuperstructureExtras(SuperstructureState state) {
    return Commands.runOnce(
        () -> {
          dispenser.setTunnelVolts(state.getValue().getTunnelVolts().getAsDouble());
          dispenser.setGripperCurrent(state.getValue().getGripperCurrent().getAsDouble());
          slam.setIntakeVolts(state.getValue().getIntakeVolts().getAsDouble());
        });
  }

  private Command getSlamCommand(Slam.Goal goal) {
    return Commands.runOnce(() -> slam.setGoal(goal));
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
    return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || dispenser.hasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  private boolean isAtGoal() {
    return elevator.isAtGoal()
        && (dispenser.isAtGoal() || Constants.getRobot() == RobotType.DEVBOT);
  }

  /** Get coral scoring state for level and algae state */
  public static SuperstructureState getScoringState(
      FieldConstants.ReefLevel height, boolean algae, boolean eject) {
    return switch (height) {
      case L1 ->
          !algae
              ? (eject ? SuperstructureState.L1_CORAL_EJECT : SuperstructureState.L1_CORAL)
              : (eject
                  ? SuperstructureState.L1_CORAL_REVERSED
                  : SuperstructureState.L1_CORAL_REVERSED_EJECT);
      case L2 ->
          !algae
              ? (eject ? SuperstructureState.L2_CORAL_EJECT : SuperstructureState.L2_CORAL)
              : (eject
                  ? SuperstructureState.L2_CORAL_REVERSED
                  : SuperstructureState.L2_CORAL_REVERSED_EJECT);
      case L3 ->
          !algae
              ? (eject ? SuperstructureState.L3_CORAL_EJECT : SuperstructureState.L3_CORAL)
              : (eject
                  ? SuperstructureState.L3_CORAL_REVERSED_EJECT
                  : SuperstructureState.L3_CORAL_REVERSED);
      case L4 ->
          !algae
              ? (eject ? SuperstructureState.L4_CORAL_EJECT : SuperstructureState.L4_CORAL)
              : (eject
                  ? SuperstructureState.L4_CORAL_REVERSED_EJECT
                  : SuperstructureState.L4_CORAL_REVERSED);
    };
  }

  public static boolean willSlam(SuperstructureState from, SuperstructureState to) {
    return from.getValue().getHeight().lowerThan(SuperstructureStateData.Height.INTAKE)
        != to.getValue().getHeight().lowerThan(SuperstructureStateData.Height.INTAKE);
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
    void accept(A a, B b, C c, D d, E e);
  }
}
