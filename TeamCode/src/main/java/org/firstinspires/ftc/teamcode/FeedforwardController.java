package org.firstinspires.ftc.teamcode;

public class FeedforwardController {

    private double kS;
    private double kV;

    public FeedforwardController(double ks, double kv) {
        kS = ks;
        kV = kv;
    }

    public double calculateWithVelocities(double nextVelocity) {
        return kS * Math.signum(nextVelocity) + kV * nextVelocity;
    }

}
