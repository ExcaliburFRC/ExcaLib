package frc.excalib.control.GenericFF;

/**
 * Interface for feedforward controllers that have kS, kV, and kA gains.
 */
public interface FeedForwardGainsSetter {
    void setKs(double Ks);
    void setKv(double Kv);
    void setKa(double Ka);
    void setKg(double Kg);
}