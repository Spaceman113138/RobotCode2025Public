// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DispenserIO {
  @AutoLog
  class DispenserIOInputs {
    public DispenserIOData data =
        new DispenserIOData(false, false, Rotation2d.kZero, Rotation2d.kZero, 0, 0, 0, 0, 0, 0);
  }

  record DispenserIOData(
      boolean motorConnected,
      boolean encoderConnected,
      Rotation2d internalPosition,
      Rotation2d encoderAbsolutePosition,
      double encoderRelativePosition,
      double velocityRadPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius) {}

  default void updateInputs(DispenserIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Rotation2d position, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
