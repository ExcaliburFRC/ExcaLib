package frc.excalib.control.GenericFF;

/**
 * Interface for feedforward controllers that have kS, kV, and kA gains.
 */
public interface FeedForwardGainsSetter {
    void setKs(double kS);
    void setKv(double kV);
    void setKa(double kA);
    void setKg(double kG);
    void setValue(double kS, double kV, double kA, double kG);

}