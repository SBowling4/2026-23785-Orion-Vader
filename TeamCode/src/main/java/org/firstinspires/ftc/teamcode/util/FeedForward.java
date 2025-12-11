package org.firstinspires.ftc.teamcode.util;

public class FeedForward {
    private double kS; // Static gain
    private double kV; // Velocity gain
    private double kA; // Acceleration gain

    public FeedForward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public FeedForward(double ks, double kv) {
        this(ks, kv, 0);
    }

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

    public double calculate(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

}
