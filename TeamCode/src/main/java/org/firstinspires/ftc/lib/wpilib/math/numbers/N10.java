// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.numbers;

import org.firstinspires.ftc.lib.wpilib.math.Nat;
import org.firstinspires.ftc.lib.wpilib.math.Num;

/** A class representing the number 0. */
public final class N10 extends Num implements Nat<N10> {
  private N10() {}

  /**
   * The integer this class represents.
   *
   * @return The literal number 0.
   */
  @Override
  public int getNum() {
    return 10;
  }

  /** The singleton instance of this class. */
  public static final N10 instance = new N10();
}
