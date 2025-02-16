// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface ReefControlsIO {
  @AutoLog
  class ReefControlsIOInputs {
    public int selectedLevel = 0; // 0 = L2, 1 = L3, 2 = L4
    public int level1State = 0; // Count
    public int level2State = 0; // Bitfield
    public int level3State = 0; // Bitfield
    public int level4State = 0; // Bitfield
    public int algaeState = (1 << 6) - 1; // Bitfield
    public boolean coopState = false; // Boolean
  }

  default void updateInputs(ReefControlsIOInputs inputs) {}

  default void setSelectedLevel(int value) {}

  default void setLevel1State(int value) {}

  default void setLevel2State(int value) {}

  default void setLevel3State(int value) {}

  default void setLevel4State(int value) {}

  default void setAlgaeState(int value) {}

  default void setCoopState(boolean value) {}
}
