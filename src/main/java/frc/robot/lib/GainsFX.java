package frc.robot.lib;

public class GainsFX {

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kV;
    public final double kA;
    public final double kS;

    public GainsFX(double _kP, double _kI, double _kD, double _kV, double _kA, double _kS) {
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kV = _kV;
        kA = _kA;
        kS = _kS;
    }
}
