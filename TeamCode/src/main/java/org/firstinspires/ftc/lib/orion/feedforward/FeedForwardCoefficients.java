package org.firstinspires.ftc.lib.orion.feedforward;

public class FeedForwardCoefficients {
    public double kS;
    public double kA;
    public double kV;

    public FeedForwardCoefficients(double kS, double kV, double kA)  {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public FeedForwardCoefficients(double kS, double kV) {
        this(kS, kV, 0);
    }

    public FeedForwardCoefficients(double kS) {
        this(kS, 0, 0);
    }

}
