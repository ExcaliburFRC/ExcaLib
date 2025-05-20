package frc.excalib.control.gains;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class GenericFF {
    public interface GenericFeedForward {
        void setKs(double Ks);
        void setKv(double Kv);
        void setKa(double Ka);
        void setKg(double Kg);
    }

    public static class ArmFF extends ArmFeedforward implements GenericFeedForward {
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

    public static class SimpleFF extends SimpleMotorFeedforward implements GenericFeedForward {
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

    public static class ElevatorFF extends ElevatorFeedforward implements GenericFeedForward {
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
