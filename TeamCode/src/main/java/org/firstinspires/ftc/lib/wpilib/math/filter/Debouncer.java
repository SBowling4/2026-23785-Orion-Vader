// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.filter;

import org.firstinspires.ftc.lib.wpilib.wpilibj.Timer;

/**
 * A simple debounce filter for boolean streams. Requires that the boolean change value from
 * baseline for a specified period of time before the filtered value changes.
 */
public class Debouncer {
  /** Type of debouncing to perform. */
  public enum DebounceType {
    /** Rising edge. */
    kRising,
    /** Falling edge. */
    kFalling,
    /** Both rising and falling edges. */
    kBoth
  }

  private double m_debounceTimeSeconds;
  private DebounceType m_debounceType;
  private boolean m_baseline;

  private double m_prevTimeSeconds;

  /**
   * Creates a new Debouncer.
   *
   * @param debounceTime The number of seconds the value must change from baseline for the filtered
   *     value to change.
   * @param type Which type of state change the debouncing will be performed on.
   */
  public Debouncer(double debounceTime, DebounceType type) {
    m_debounceTimeSeconds = debounceTime;
    m_debounceType = type;

    resetTimer();

    m_baseline =
        switch (m_debounceType) {
          case kBoth, kRising -> false;
          case kFalling -> true;
        };
  }

  /**
   * Creates a new Debouncer. Baseline value defaulted to "false."
   *
   * @param debounceTime The number of seconds the value must change from baseline for the filtered
   *     value to change.
   */
  public Debouncer(double debounceTime) {
    this(debounceTime, DebounceType.kRising);
  }

  private void resetTimer() {
    m_prevTimeSeconds = Timer.getTimestampSeconds();
  }

  private boolean hasElapsed() {
    return Timer.getTimestampSeconds() - m_prevTimeSeconds >= m_debounceTimeSeconds;
  }

  /**
   * Applies the debouncer to the input stream.
   *
   * @param input The current value of the input stream.
   * @return The debounced value of the input stream.
   */
  public boolean calculate(boolean input) {
    if (input == m_baseline) {
      resetTimer();
    }

    if (hasElapsed()) {
      if (m_debounceType == DebounceType.kBoth) {
        m_baseline = input;
        resetTimer();
      }
      return input;
    } else {
      return m_baseline;
    }
  }

  /**
   * Sets the time to debounce.
   *
   * @param time The number of seconds the value must change from baseline for the filtered value to
   *     change.
   */
  public void setDebounceTime(double time) {
    m_debounceTimeSeconds = time;
  }

  /**
   * Gets the time to debounce.
   *
   * @return The number of seconds the value must change from baseline for the filtered value to
   *     change.
   */
  public double getDebounceTime() {
    return m_debounceTimeSeconds;
  }

  /**
   * Sets the debounce type.
   *
   * @param type Which type of state change the debouncing will be performed on.
   */
  public void setDebounceType(DebounceType type) {
    m_debounceType = type;
  }

  /**
   * Gets the debounce type.
   *
   * @return Which type of state change the debouncing will be performed on.
   */
  public DebounceType getDebounceType() {
    return m_debounceType;
  }
}
