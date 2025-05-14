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
        public void setKs(double kS) {
            super.setKs(kS);
        }


        @Override
        public void setKv(double kV) {
            super.setKv(kV);
        }


        @Override
        public void setKa(double kA) {
            super.setKa(kA);
        }


        @Override
        public void setKg(double kG) {
            super.setKg(kG);
        }

        @Override
        public void setValue(double kS, double kV, double kA, double kG) {
            setKa(kA);
            setKs(kS);
            setKv(kV);
            setKg(kG);
        }
    }

    public static class SimpleFF extends SimpleMotorFeedforward implements FeedForwardGainsSetter {
        public SimpleFF() {
            super(0, 0, 0);
        }


        @Override
        public void setKs(double kS) {
            super.setKs(kS);
        }


        @Override
        public void setKv(double kV) {
            super.setKv(kV);
        }


        @Override
        public void setKa(double kA) {
            super.setKa(kA);
        }


        @Override
        public void setKg(double kG) {
            return;
        }

        @Override
        public void setValue(double kS, double kV, double kA, double kG) {
            if (kG != 0) {
                throw new IllegalArgumentException("SimpleMotorFeedforward does not support gravity gain (kG), please make it zero!");
            }
            setKa(kA);
            setKs(kS);
            setKv(kV);
            setKg(kG);
        }
    }

    public static class ElevatorFF extends ElevatorFeedforward implements FeedForwardGainsSetter {
        public ElevatorFF() {
            super(0, 0, 0, 0);
        }


        @Override
        public void setKs(double kS) {
            super.setKs(kS);
        }


        @Override
        public void setKv(double kV) {
            super.setKv(kV);
        }


        @Override
        public void setKa(double kA) {
            super.setKa(kA);
        }


        @Override
        public void setKg(double kG) {
            super.setKg(kG);
        }

        @Override
        public void setValue(double kS, double kV, double kA, double kG) {
            setKa(kA);
            setKs(kS);
            setKv(kV);
            setKg(kG);
        }

    }
}
