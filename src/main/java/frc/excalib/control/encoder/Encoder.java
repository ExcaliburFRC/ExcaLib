package frc.excalib.control.encoder;

public interface Encoder {

    void setPositionConversionFactor(double positionConversionFactor);

    void setVelocityConversionFactor(double velocityConversionFactor);

    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setInverted(boolean inverted);
}
