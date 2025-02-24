// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public boolean motorConnected = false;
    public boolean followerConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;

    public double[] appliedVoltage = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
