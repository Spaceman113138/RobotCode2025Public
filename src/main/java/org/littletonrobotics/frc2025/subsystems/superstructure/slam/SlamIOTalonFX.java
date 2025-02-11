// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.slam;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Celsius;
import static org.littletonrobotics.frc2025.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

public class SlamIOTalonFX implements SlamIO {
  static double reduction = 21.0;

  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public SlamIOTalonFX() {
    talon = new TalonFX(0, "*");

    var slamConfig = new TalonFXConfiguration();
    slamConfig.Feedback.RotorToSensorRatio = reduction;
    slamConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    slamConfig.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    slamConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    slamConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    slamConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(slamConfig, 0.25));

    // Slam
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  @Override
  public void updateInputs(SlamIOInputs inputs) {
    // Refresh status signals and check if hardware connected
    boolean slamMotorConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp)
            .isOK();
    inputs.connected = motorConnectedDebouncer.calculate(slamMotorConnected);
    inputs.positionRad = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void runTorqueCurrent(double current) {
    talon.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              tryUntilOk(
                  5,
                  () ->
                      talon.setNeutralMode(
                          enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25));
            })
        .start();
  }
}
