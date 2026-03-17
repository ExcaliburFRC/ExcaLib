package frc.excalib.control.math.periodics;

public class PeriodicTask {
    private Runnable task;
    PeriodicScheduler.PERIOD period;

    public PeriodicTask(Runnable toRun, PeriodicScheduler.PERIOD period){
        this.task = toRun;
        this.period = period;
        this.period.add(this);
    }

    public void execute(){
        task.run();
    }
    public void setTask(Runnable task){
        this.task = task;
    }
}