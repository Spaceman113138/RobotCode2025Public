// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.slam;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
  @AutoLog
  class SlamIOInputs {
    public boolean connected = true;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(SlamIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
