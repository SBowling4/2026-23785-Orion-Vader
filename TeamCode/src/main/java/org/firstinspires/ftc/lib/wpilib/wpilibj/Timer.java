// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.wpilibj;

/**
 * A timer class.
 *
 * <p>Note that if the user calls SimHooks.restartTiming(), they should also reset the timer so
 * get() won't return a negative duration.
 */
public class Timer {
  /**
   * Pause the thread for a specified time. Pause the execution of the thread for a specified period
   * of time given in seconds. Motors will continue to run at their last assigned values, and
   * sensors will continue to update. Only the task containing the wait will pause until the wait
   * time is expired.
   *
   * @param seconds Length of time to pause
   */
  public static void delay(final double seconds) {
    try {
      Thread.sleep((long) (seconds * 1e3));
    } catch (final InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }

  private double m_startTime;
  private double m_accumulatedTime;
  private boolean m_running;

  /** Timer constructor. */
  public Timer() {
    reset();
  }

  private double getMsClock() {
    return getTimestampSeconds() * 1000;
  }

  public static double getTimestampSeconds() {
    return System.nanoTime() / 1e9;
  }

  /**
   * Get the current time from the timer. If the clock is running it is derived from the current
   * system clock the start time stored in the timer class. If the clock is not running, then return
   * the time when it was last stopped.
   *
   * @return Current time value for this timer in seconds
   */
  public double get() {
    if (m_running) {
      return m_accumulatedTime + (getMsClock() - m_startTime) / 1000.0;
    } else {
      return m_accumulatedTime;
    }
  }

  /**
   * Reset the timer by setting the time to 0.
   *
   * <p>Make the timer startTime the current time so new requests will be relative now.
   */
  public final void reset() {
    m_accumulatedTime = 0;
    m_startTime = getMsClock();
  }

  /**
   * Start the timer running. Just set the running flag to true indicating that all time requests
   * should be relative to the system clock. Note that this method is a no-op if the timer is
   * already running.
   */
  public void start() {
    if (!m_running) {
      m_startTime = getMsClock();
      m_running = true;
    }
  }

  /**
   * Restart the timer by stopping the timer, if it is not already stopped, resetting the
   * accumulated time, then starting the timer again. If you want an event to periodically reoccur
   * at some time interval from the start time, consider using advanceIfElapsed() instead.
   */
  public void restart() {
    if (m_running) {
      stop();
    }
    reset();
    start();
  }

  /**
   * Stop the timer. This computes the time as of now and clears the running flag, causing all
   * subsequent time requests to be read from the accumulated time rather than looking at the system
   * clock.
   */
  public void stop() {
    m_accumulatedTime = get();
    m_running = false;
  }

  /**
   * Check if the period specified has passed.
   *
   * @param seconds The period to check.
   * @return Whether the period has passed.
   */
  public boolean hasElapsed(double seconds) {
    return get() >= seconds;
  }

  /**
   * Check if the period specified has passed and if it has, advance the start time by that period.
   * This is useful to decide if it's time to do periodic work without drifting later by the time it
   * took to get around to checking.
   *
   * @param seconds The period to check.
   * @return Whether the period has passed.
   */
  public boolean advanceIfElapsed(double seconds) {
    if (get() >= seconds) {
      // Advance the start time by the period.
      // Don't set it to the current time... we want to avoid drift.
      m_startTime += seconds * 1000;
      return true;
    } else {
      return false;
    }
  }

  /**
   * Whether the timer is currently running.
   *
   * @return true if running.
   */
  public boolean isRunning() {
    return m_running;
  }
}
