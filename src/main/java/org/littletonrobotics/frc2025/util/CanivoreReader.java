// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.wpilibj.Notifier;
import java.util.Optional;
import org.littletonrobotics.frc2025.Constants;

public class CanivoreReader {
  private final CANBus canBus;
  private final Notifier notifier;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanivoreReader(String canBusName) {
    canBus = new CANBus(canBusName);
    notifier =
        new Notifier(
            () -> {
              var statusTemp = Optional.of(canBus.getStatus());
              synchronized (this) {
                status = statusTemp;
              }
            });
    notifier.startPeriodic(Constants.loopPeriodSecs);
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}
