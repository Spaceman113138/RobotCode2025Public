// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import static edu.wpi.first.units.Units.*;
import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class DispenserIOTalonFX implements DispenserIO {
  public static final double reduction = 3.0;
  private static final Rotation2d offset = new Rotation2d();
  private static final int encoderId = 0;

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration Config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> internalPosition;
  private final StatusSignal<Angle> encoderAbsolutePosition;
  private final StatusSignal<Angle> encoderRelativePosition;
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public DispenserIOTalonFX() {
    talon = new TalonFX(0, "*");
    encoder = new CANcoder(encoderId, "*");

    // Configure  motor
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    Config.Feedback.RotorToSensorRatio = reduction;
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Feedback.FeedbackRemoteSensorID = encoderId;
    Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    Config.Feedback.SensorToMechanismRatio = 1.0;
    Config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    Config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    Config.CurrentLimits.StatorCurrentLimit = 40.0;
    Config.CurrentLimits.StatorCurrentLimitEnable = true;
    Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config, 0.25));

    // Configure encoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = offset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

    // Get and set status signals
    internalPosition = talon.getPosition();
    encoderAbsolutePosition = encoder.getAbsolutePosition();
    encoderRelativePosition = encoder.getPosition();
    Velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        internalPosition,
        encoderAbsolutePosition,
        encoderRelativePosition,
        Velocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
    ParentDevice.optimizeBusUtilizationForAll(talon, encoder);
  }

  @Override
  public void updateInputs(DispenserIOInputs inputs) {
    // Refresh status signals and check if hardware connected
    boolean motorConnected =
        BaseStatusSignal.refreshAll(
                internalPosition,
                Velocity,
                appliedVolts,
                supplyCurrentAmps,
                torqueCurrentAmps,
                temp)
            .isOK();
    boolean encoderConnected =
        BaseStatusSignal.refreshAll(encoderAbsolutePosition, encoderRelativePosition).isOK();

    inputs.motorConnected = motorConnectedDebouncer.calculate(motorConnected);
    inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoderConnected);
    // Pivot inputs
    inputs.internalPosition = Rotation2d.fromRotations(internalPosition.getValueAsDouble());
    inputs.encoderAbsolutePosition =
        Rotation2d.fromRotations(encoderAbsolutePosition.getValueAsDouble()).minus(offset);
    inputs.encoderRelativePosition =
        encoderRelativePosition.getValue().in(Radians) - offset.getRadians();
    inputs.velocityRadPerSec = Velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    Config.Slot0.kP = kP;
    Config.Slot0.kI = kI;
    Config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              Config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(Config));
            })
        .start();
  }
}
