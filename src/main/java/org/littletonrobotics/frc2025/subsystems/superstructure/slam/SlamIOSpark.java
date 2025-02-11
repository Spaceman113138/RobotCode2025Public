// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.slam;

import static org.littletonrobotics.frc2025.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class SlamIOSpark implements SlamIO {
  private static final double reduction = 1.0;
  private static final boolean inverted = false;
  private int currentLimit = 40;

  private final SparkBase spark;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig config;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private boolean brakeModeEnabled = true;

  public SlamIOSpark() {
    spark = new SparkMax(7, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();

    config = new SparkMaxConfig();
    config
        .idleMode(
            brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .inverted(inverted)
        .smartCurrentLimit(currentLimit);
    config
        .encoder
        .positionConversionFactor(reduction)
        .velocityConversionFactor(reduction)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(SlamIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, position -> inputs.positionRad = position);
    ifOk(spark, encoder::getVelocity, velocity -> inputs.velocityRadPerSec = velocity);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getBusVoltage, spark::getAppliedOutput},
        x -> inputs.appliedVolts = x[0] * x[1]);
    ifOk(spark, spark::getOutputCurrent, current -> inputs.torqueCurrentAmps = current);
    inputs.connected = connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void runTorqueCurrent(double current) {
    if (currentLimit != Math.abs((int) current)) {
      // Reflash spark with current limit
      currentLimit = Math.abs((int) current);
      new Thread(
              () ->
                  tryUntilOk(
                      spark,
                      5,
                      () ->
                          spark.configure(
                              config.smartCurrentLimit(currentLimit),
                              SparkBase.ResetMode.kNoResetSafeParameters,
                              SparkBase.PersistMode.kNoPersistParameters)))
          .start();
    }
    spark.set(Math.signum(current));
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    new Thread(
            () ->
                tryUntilOk(
                    spark,
                    5,
                    () ->
                        spark.configure(
                            config.idleMode(
                                brakeModeEnabled
                                    ? SparkBaseConfig.IdleMode.kBrake
                                    : SparkBaseConfig.IdleMode.kCoast),
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters)))
        .start();
  }
}
