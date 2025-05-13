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
    }
}
