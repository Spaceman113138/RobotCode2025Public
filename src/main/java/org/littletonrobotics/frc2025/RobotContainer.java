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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.commands.AutoScore;
import org.littletonrobotics.frc2025.commands.DriveCommands;
import org.littletonrobotics.frc2025.commands.DriveTrajectory;
import org.littletonrobotics.frc2025.commands.IntakeCommands;
import org.littletonrobotics.frc2025.subsystems.drive.*;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOSim;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOSpark;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.DispenserIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.DispenserIOTalonFX;
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
import org.littletonrobotics.frc2025.util.TriggerUtil;
import org.littletonrobotics.frc2025.util.trajectory.HolonomicTrajectory;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private final Superstructure superstructure;
  private RollerSystem funnel;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);
  private final Trigger superstructureDisable = overrides.driverSwitch(1);
  private final Trigger superstructureCoast = overrides.driverSwitch(2);
  private final Trigger aprilTagsBlueReef = overrides.multiDirectionSwitchLeft();
  private final Trigger aprilTagsRedReef = overrides.multiDirectionSwitchRight();
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);

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
                  this::getSelectedAprilTagLayout,
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 0),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 1),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 2),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 3));
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
          dispenser =
              new Dispenser(
                  new DispenserIO() {}, new RollerSystemIOSpark(5, true), new RollerSystemIO() {});
          slam =
              new Slam(new SlamIOSpark(), new RollerSystemIOTalonFX(6, "", 40, false, false, 1.0));
          funnel = new RollerSystem("Funnel", new RollerSystemIOSpark(4, false));
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
                    this::getSelectedAprilTagLayout,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {});
        case DEVBOT -> vision = new Vision(this::getSelectedAprilTagLayout, new VisionIO() {});
        default -> vision = new Vision(this::getSelectedAprilTagLayout);
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
    if (funnel == null) {
      funnel = new RollerSystem("Funnel", new RollerSystemIO() {});
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
    superstructure.setDisabledOverride(superstructureDisable);
    elevator.setOverrides(() -> elevatorCoastOverride, superstructureDisable);
    dispenser.setOverrides(() -> elevatorCoastOverride, superstructureDisable);
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
    superstructureCoast
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
            () -> -driver.getRightX() - operator.getRightX(),
            robotRelative.getAsBoolean()));

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

    driver
        .rightTrigger()
        .whileTrue(
            new AutoScore(
                drive,
                superstructure,
                () -> new FieldConstants.CoralObjective(1, FieldConstants.ReefHeight.L4),
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()))
        .onFalse(
            superstructure
                .runGoal(superstructure::getState)
                .until(
                    () -> {
                      var robot =
                          AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
                      return robot.getTranslation().getDistance(FieldConstants.Reef.center)
                          >= FieldConstants.Reef.faceLength
                              + DriveConstants.robotWidth / 2.0
                              + AutoScore.minDistanceReefClear.get();
                    }));

    // Operator command for algae intake
    operator
        .leftBumper()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.ALGAE_FLOOR_INTAKE)
                .withName("Algae Floor Intake"));

    // Operator command for coral intake
    operator.leftTrigger().whileTrue(IntakeCommands.intake(superstructure, funnel));

    // Operator commands for superstructure
    bindOperatorCoralCommand(
        operator.a(), SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT);
    bindOperatorCoralCommand(
        operator.x(), SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT);
    bindOperatorCoralCommand(
        operator.b(), SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT);
    bindOperatorCoralCommand(
        operator.y(), SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT);
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());

    // AprilTag layout alert
    boolean aprilTagAlertActive = getSelectedAprilTagLayout() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use (" + getSelectedAprilTagLayout().toString() + ").");
    }
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsBlueReef.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.BLUE_REEF;
    } else if (aprilTagsRedReef.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.RED_REEF;
    } else {
      return FieldConstants.defaultAprilTagType;
    }
  }

  private void bindOperatorCoralCommand(
      Trigger faceButton,
      SuperstructureState coralReadyState,
      SuperstructureState coralEjectState) {
    faceButton.whileTrueContinuous(
        superstructure
            .runGoal(coralReadyState)
            .withName("Operator: " + coralReadyState.toString()));
    faceButton
        .and(operator.rightTrigger())
        .whileTrueContinuous(
            superstructure
                .runGoal(coralEjectState)
                .withName("Operator: " + coralEjectState.toString()));
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
