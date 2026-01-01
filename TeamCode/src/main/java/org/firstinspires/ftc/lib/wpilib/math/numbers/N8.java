// Copyright (c) 2025-2026 FTC 8696
// All rights reserved.

package org.firstinspires.ftc.lib.wpilib.math.numbers;

import org.firstinspires.ftc.lib.wpilib.math.Nat;
import org.firstinspires.ftc.lib.wpilib.math.Num;

/** A class representing the number 0. */
public final class N8 extends Num implements Nat<N8> {
  private N8() {}

  /**
   * The integer this class represents.
   *
   * @return The literal number 0.
   */
  @Override
  public int getNum() {
    return 8;
  }

  /** The singleton instance of this class. */
  public static final N8 instance = new N8();
}
