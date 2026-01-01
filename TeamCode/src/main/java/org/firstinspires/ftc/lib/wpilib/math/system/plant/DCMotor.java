// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.system.plant;

import org.firstinspires.ftc.lib.wpilib.math.util.Units;

/** Holds the constants for a DC motor. */
public class DCMotor {
  /** Voltage at which the motor constants were measured. */
  public final double nominalVoltageVolts;

  /** Torque when stalled. */
  public final double stallTorqueNewtonMeters;

  /** Current draw when stalled. */
  public final double stallCurrentAmps;

  /** Current draw under no load. */
  public final double freeCurrentAmps;

  /** Angular velocity under no load. */
  public final double freeSpeedRadPerSec;

  /** Motor internal resistance. */
  public final double rOhms;

  /** Motor velocity constant. */
  public final double KvRadPerSecPerVolt;

  /** Motor torque constant. */
  public final double KtNMPerAmp;

  /**
   * Constructs a DC motor.
   *
   * @param nominalVoltageVolts Voltage at which the motor constants were measured.
   * @param stallTorqueNewtonMeters Torque when stalled.
   * @param stallCurrentAmps Current draw when stalled.
   * @param freeCurrentAmps Current draw under no load.
   * @param freeSpeedRadPerSec Angular velocity under no load.
   * @param numMotors Number of motors in a gearbox.
   */
  public DCMotor(
      double nominalVoltageVolts,
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRadPerSec,
      int numMotors) {
    this.nominalVoltageVolts = nominalVoltageVolts;
    this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
    this.stallCurrentAmps = stallCurrentAmps * numMotors;
    this.freeCurrentAmps = freeCurrentAmps * numMotors;
    this.freeSpeedRadPerSec = freeSpeedRadPerSec;

    this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
    this.KvRadPerSecPerVolt =
        freeSpeedRadPerSec / (nominalVoltageVolts - rOhms * this.freeCurrentAmps);
    this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
  }

  /**
   * Calculate current drawn by motor with given speed and input voltage.
   *
   * @param speedRadiansPerSec The current angular velocity of the motor.
   * @param voltageInputVolts The voltage being applied to the motor.
   * @return The estimated current.
   */
  public double getCurrent(double speedRadiansPerSec, double voltageInputVolts) {
    return -1.0 / KvRadPerSecPerVolt / rOhms * speedRadiansPerSec + 1.0 / rOhms * voltageInputVolts;
  }

  /**
   * Calculate current drawn by motor for a given torque.
   *
   * @param torqueNm The torque produced by the motor.
   * @return The current drawn by the motor.
   */
  public double getCurrent(double torqueNm) {
    return torqueNm / KtNMPerAmp;
  }

  /**
   * Calculate torque produced by the motor with a given current.
   *
   * @param currentAmpere The current drawn by the motor.
   * @return The torque output.
   */
  public double getTorque(double currentAmpere) {
    return currentAmpere * KtNMPerAmp;
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param torqueNm The torque produced by the motor.
   * @param speedRadiansPerSec The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public double getVoltage(double torqueNm, double speedRadiansPerSec) {
    return 1.0 / KvRadPerSecPerVolt * speedRadiansPerSec + 1.0 / KtNMPerAmp * rOhms * torqueNm;
  }

  /**
   * Calculates the angular speed produced by the motor at a given torque and input voltage.
   *
   * @param torqueNm The torque produced by the motor.
   * @param voltageInputVolts The voltage applied to the motor.
   * @return The angular speed of the motor.
   */
  public double getSpeed(double torqueNm, double voltageInputVolts) {
    return voltageInputVolts * KvRadPerSecPerVolt
        - 1.0 / KtNMPerAmp * torqueNm * rOhms * KvRadPerSecPerVolt;
  }

  /**
   * Returns a copy of this motor with the given gearbox reduction applied.
   *
   * @param gearboxReduction The gearbox reduction.
   * @return A motor with the gearbox reduction applied.
   */
  public DCMotor withReduction(double gearboxReduction) {
    return new DCMotor(
        nominalVoltageVolts,
        stallTorqueNewtonMeters * gearboxReduction,
        stallCurrentAmps,
        freeCurrentAmps,
        freeSpeedRadPerSec / gearboxReduction,
        1);
  }

  public static DCMotor GOBILDA_5203_6000RPM(int numMotors) {
    return new DCMotor(
        12, .1442, 9.2, .25, Units.rotationsPerMinuteToRadiansPerSecond(6000), numMotors);
  }

  public static DCMotor GOBILDA_5203_1620RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors).withReduction(1 + (46.0 / 17.0));
  }

  public static DCMotor GOBILDA_5203_1150RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors).withReduction(1 + (46.0 / 11.0));
  }

  public static DCMotor GOBILDA_5203_435RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors).withReduction((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)));
  }

  public static DCMotor GOBILDA_5203_312RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors).withReduction((1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)));
  }

  public static DCMotor GOBILDA_5203_223RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors).withReduction((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
  }

  public static DCMotor GOBILDA_5203_117RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors)
        .withReduction((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)));
  }

  public static DCMotor GOBILDA_5203_84RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors)
        .withReduction((1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)));
  }

  public static DCMotor GOBILDA_5203_60RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors)
        .withReduction((1 + (46.0 / 17.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
  }

  public static DCMotor GOBILDA_5203_43RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors)
        .withReduction((1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)) * (1 + (46.0 / 11.0)));
  }

  public static DCMotor GOBILDA_5203_30RPM(int numMotors) {
    return GOBILDA_5203_6000RPM(numMotors)
        .withReduction(
            (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)) * (1 + (46.0 / 17.0)));
  }
}
