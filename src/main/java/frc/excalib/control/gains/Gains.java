package frc.excalib.control.gains;

import edu.wpi.first.math.controller.PIDController;


/**
 * This class bundles together multiple types of gains, including PID gains and feedforward gains.
 */
public class Gains {
    // Proportional, Integral, and Derivative gains for PID control
    public double kp, ki, kd;
    // Feedforward gains: static, gravity, velocity, and acceleration
    public double ks, kg, kv, ka;


    /**
     * Constructs a Gains object with specified PID and feedforward gains.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     * @param ks The static gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     * @param kg The gravity gain.
     */
    public Gains(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }


    /**
     * Constructs a Gains object with only PID gains.
     * Feedforward gains are set to 0.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    public Gains(double kp, double ki, double kd) {
        this(kp, ki, kd, 0, 0, 0, 0);
    }


    /**
     * Constructs a Gains object with only feedforward gains.
     * PID gains are set to 0.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     */
    public Gains(double ks, double kg, double kv, double ka) {
        this(0, 0, 0, ks, kv, ka, kg);
    }


    /**
     * Constructs a Gains object by combining PID gains and feedforward gains from two separate Gains objects.
     *
     * @param PIDgains The Gains object containing PID gains.
     * @param FFgains The Gains object containing feedforward gains.
     */
    public Gains(Gains PIDgains, Gains FFgains) {
        this(PIDgains.kp, PIDgains.ki, PIDgains.kd, FFgains.ks, FFgains.kv, FFgains.ka, FFgains.kg);
    }


    /**
     * Constructs a Gains object with all gains set to 0.
     */
    public Gains() {
        this(0, 0, 0, 0, 0, 0, 0);
    }


    /**
     * Constructs a Gains object by copying the values from another Gains object.
     *
     * @param gains The Gains object to copy.
     */
    public Gains(Gains gains) {
        this(gains.kp, gains.ki, gains.kd, gains.ks, gains.kg, gains.kv, gains.ka);
    }


    /**
     * Sets the PID gains.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    public void setPIDgains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }


    /**
     * Sets the feedforward gains.
     *
     * @param ks The static gain.
     * @param kg The gravity gain.
     * @param kv The velocity gain.
     * @param ka The acceleration gain.
     */
    public void setFeedForwardGains(double ks, double kg, double kv, double ka) {
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }


    public PIDController getPIDcontroller(){
        return new PIDController(this.kp, this.ki, this.kd);
    }

    public <GenericFeedFoward extends GenericFF.GenericFeedForward> GenericFeedFoward applyGains(GenericFeedFoward feedForward){
        feedForward.setKv(this.kv);
        feedForward.setKs(this.ks);
        feedForward.setKa(this.ka);
        feedForward.setKg(this.kg);
        return feedForward;
    }
}
