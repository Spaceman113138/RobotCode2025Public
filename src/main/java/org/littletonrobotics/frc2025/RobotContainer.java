// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.frc2025.commands.DriveCommands;
import org.littletonrobotics.frc2025.commands.DriveTrajectory;
import org.littletonrobotics.frc2025.subsystems.drive.*;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOSim;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOSim;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.superstructure.slam.*;
import org.littletonrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIO;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIONorthstar;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.OverrideSwitches;
import org.littletonrobotics.frc2025.util.trajectory.HolonomicTrajectory;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Load RobotState class
  private final RobotState robotState = RobotState.getInstance();

  // Subsystems
  private Drive drive;
  private Vision vision;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger elevatorDisable = overrides.driverSwitch(1);
  private final Trigger elevatorCoast = overrides.driverSwitch(2);

  private boolean elevatorCoastOverride = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Elevator elevator = null;
    Dispenser dispenser = null;
    Slam slam = null;
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[0]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[1]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[2]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[3]));
          vision =
              new Vision(
                  new VisionIONorthstar(0),
                  new VisionIONorthstar(1),
                  new VisionIONorthstar(2),
                  new VisionIONorthstar(3));
          elevator = new Elevator(new ElevatorIOTalonFX());
          dispenser =
              new Dispenser(
                  new DispenserIOTalonFX(),
                  new RollerSystemIOTalonFX(0, "*", 0, false, false, 1.0),
                  new RollerSystemIOTalonFX(0, "*", 0, false, false, 1.0));
          slam =
              new Slam(
                  new SlamIOTalonFX(), new RollerSystemIOTalonFX(0, "*", 0, false, false, 1.0));
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIORedux(),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[0]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[1]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[2]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[3]));
          elevator = new Elevator(new ElevatorIOTalonFX());
          slam =
              new Slam(new SlamIOSpark(), new RollerSystemIOTalonFX(6, "", 40, false, false, 1.0));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          dispenser =
              new Dispenser(
                  new DispenserIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2));
          slam =
              new Slam(
                  new SlamIOSim(), new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.02));
        }
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      switch (Constants.getRobot()) {
        case COMPBOT ->
            vision =
                new Vision(
                    new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        case DEVBOT -> vision = new Vision(new VisionIO() {});
        default -> vision = new Vision();
      }
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (dispenser == null) {
      dispenser =
          new Dispenser(new DispenserIO() {}, new RollerSystemIO() {}, new RollerSystemIO() {});
    }
    if (slam == null) {
      slam = new Slam(new SlamIO() {}, new RollerSystemIO() {});
    }
    superstructure = new Superstructure(elevator, dispenser, slam);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up Characterization routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    HolonomicTrajectory testTrajectory = new HolonomicTrajectory("BLOB");
    autoChooser.addOption(
        "Drive Trajectory",
        Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(AllianceFlipUtil.apply(testTrajectory.getStartPose())))
            .andThen(new DriveTrajectory(drive, testTrajectory)));
    autoChooser.addOption("Elevator static", elevator.staticCharacterization(2.0));
    autoChooser.addOption("Pivot static", dispenser.staticCharacterization(2.0));

    // Set up overrides
    superstructure.setDisabledOverride(elevatorDisable);
    elevator.setOverrides(() -> elevatorCoastOverride, elevatorDisable);
    dispenser.setOverrides(() -> elevatorCoastOverride, elevatorDisable);
    slam.setCoastOverride(() -> elevatorCoastOverride);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    elevatorCoast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        elevatorCoastOverride = true;
                      }
                    })
                .ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> elevatorCoastOverride = false).ignoringDisable(true));

    RobotModeTriggers.disabled()
        .onFalse(Commands.runOnce(() -> elevatorCoastOverride = false).ignoringDisable(true));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY() - operator.getLeftY(),
            () -> -driver.getLeftX() - operator.getLeftX(),
            () -> -driver.getRightX() - operator.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(new Rotation2d()))),
                    drive)
                .ignoringDisable(true));

    // Operator command for algae intake
    operator
        .leftBumper()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.State.ALGAE_FLOOR_INTAKE.getValue())
                .withName("Algae Floor Intake"));

    // Operator commands for superstructure
    operator
        .a()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.State.L1_CORAL.getValue())
                .withName("Scoring L1 Coral"));
    operator
        .x()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.State.L2_CORAL.getValue())
                .withName("Scoring L2 Coral"));
    operator
        .b()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.State.L3_CORAL.getValue())
                .withName("Scoring L3 Coral"));
    operator
        .y()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.State.L4_CORAL.getValue())
                .withName("Scoring L4 Coral"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
