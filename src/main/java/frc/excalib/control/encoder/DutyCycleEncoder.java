package frc.excalib.control.encoder;

public class DutyCycleEncoder extends edu.wpi.first.wpilibj.DutyCycleEncoder implements Encoder {
    private double positionConversionFactor, velocityConversionFactor;
    private double position;

    public DutyCycleEncoder(int port){
        super(port);
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
        this.positionConversionFactor = conversionFactor;
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
        this.velocityConversionFactor = conversionFactor;
    }

    @Override
    public double getPosition() {
        return super.get() * this.positionConversionFactor + this.position;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void setPosition(double position, boolean offset) {
        this.position = offset? -position : position;
    }
}
