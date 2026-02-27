package org.firstinspires.ftc.lib.orion.feedforward;

/**
 * FeedForward class to calculate feedforward control outputs based on static, velocity, and acceleration gains.
 */
public class FeedForward {
    private double kS; // Static gain
    private double kV; // Velocity gain
    private double kA; // Acceleration gain

    public record FeedForwardCoefficients(double kS, double kV, double kA) {}

    /**
     * Constructor for FeedForward with all gains.
     *
     * @param kS Static gain.
     * @param kV Velocity gain.
     * @param kA Acceleration gain.
     */
    public FeedForward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Constructor for FeedForward with static and velocity gains only.
     *
     * @param ks Static gain.
     * @param kv Velocity gain.
     */
    public FeedForward(double ks, double kv) {
        this(ks, kv, 0);
    }

    /**
     * Constructor for FeedForward with only static gain.
     * @param ks
     */
    public FeedForward(double ks) {
        this(ks, 0, 0);
    }

    public FeedForward(FeedForwardCoefficients coefficients) {
        this(coefficients.kS, coefficients.kV, coefficients.kA);
    }

    /**
     * Default constructor for FeedForward with all gains set to zero.
     */
    public FeedForward() {
        this(0, 0, 0);
    }

    public double getkS() {
        return kS;
    }

    public void setkS(double kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    /**
     * Calculate the feedforward output based on velocity and acceleration.
     *
     * @param velocity    The target velocity.
     * @param acceleration The target acceleration.
     * @return The calculated feedforward output.
     */
    public double calculate(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    /**
     * Calculate the feedforward output based on velocity only.
     *
     * @param velocity The target velocity.
     * @return The calculated feedforward output.
     */
    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    /**
     * Calculate the feedforward output with zero velocity and acceleration.
     *
     * @return The calculated feedforward output.
     */
    public double calculate() {
        return calculate(0, 0);
    }

}
