// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] allSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(BaseStatusSignal... signals) {
    BaseStatusSignal[] newSignals = new BaseStatusSignal[allSignals.length + signals.length];
    System.arraycopy(allSignals, 0, newSignals, 0, allSignals.length);
    System.arraycopy(signals, 0, newSignals, allSignals.length, signals.length);
    allSignals = newSignals;
  }

  /** Refresh all regisstered signals. */
  public static void refreshAll() {
    if (allSignals.length > 0) {
      BaseStatusSignal.refreshAll(allSignals);
    }
  }
}
