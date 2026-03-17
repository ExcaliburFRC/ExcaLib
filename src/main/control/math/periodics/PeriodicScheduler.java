package frc.excalib.control.math.periodics;

import java.util.ArrayList;

public class PeriodicScheduler {

    private static final PeriodicScheduler instance = new PeriodicScheduler();

    private PeriodicScheduler() {}

    public enum PERIOD {
        MILLISECONDS_10(10),
        MILLISECONDS_20(20),
        MILLISECONDS_50(50),
        MILLISECONDS_100(100),
        MILLISECONDS_500(500),
        SECOND(1000);
        private final ArrayList<PeriodicTask> tasks;
        public final int milliseconds;

        PERIOD(int milliseconds) {
            this.tasks = new ArrayList<>();
            this.milliseconds = milliseconds;
        }

        public void run() {
            for (PeriodicTask task : this.tasks) task.execute();
        }

        public void add(PeriodicTask task) {
            this.tasks.add(task);
        }

    }

     public static PeriodicScheduler getInstance() {
        return instance;
    }
}