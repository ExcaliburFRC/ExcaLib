package frc.excalib.control.GenericFF;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class GenericFF {
    public static class ArmFF extends ArmFeedforward implements FeedForwardGainsSetter {
        public ArmFF() {
            super(0, 0, 0, 0);
        }

        @Override
        public void setKs(double Ks) {
            super.setKs(Ks);
        }


        @Override
        public void setKv(double Kv) {
            super.setKv(Kv);
        }


        @Override
        public void setKa(double Ka) {
            super.setKa(Ka);
        }


        @Override
        public void setKg(double Kg) {
            super.setKg(Kg);
        }
    }

    public static class SimpleFF extends SimpleMotorFeedforward implements FeedForwardGainsSetter {
        public SimpleFF() {
            super(0, 0, 0);
        }


        @Override
        public void setKs(double Ks) {
            super.setKs(Ks);
        }


        @Override
        public void setKv(double Kv) {
            super.setKv(Kv);
        }


        @Override
        public void setKa(double Ka) {
            super.setKa(Ka);
        }


        @Override
        public void setKg(double Kg) {
            if (Kg != 0) throw new IllegalArgumentException("SimpleMotorFeedforward does not support gravity gain (Kg), please make it zero!");

            return;
        }
    }

    public static class ElevatorFF extends ElevatorFeedforward implements FeedForwardGainsSetter {
        public ElevatorFF() {
            super(0, 0, 0, 0);
        }


        @Override
        public void setKs(double Ks) {
            super.setKs(Ks);
        }


        @Override
        public void setKv(double Kv) {
            super.setKv(Kv);
        }


        @Override
        public void setKa(double Ka) {
            super.setKa(Ka);
        }


        @Override
        public void setKg(double Kg) {
            super.setKg(Kg);
        }
    }
}
