// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public RollerSystemIOData data = new RollerSystemIOData(0, 0, 0, 0, 0, 0, false);
  }

  record RollerSystemIOData(
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean connected) {}
  ;

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  /* Run rollers at volts */
  default void runVolts(double volts) {}

  /* Stop rollers */
  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
