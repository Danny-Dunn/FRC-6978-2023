package frc.robot.Framework;

import java.util.ArrayList;
import java.util.List;

public class Scheduler {
    List<IPeriodicTask> tasks = new ArrayList<IPeriodicTask>();

    public void add(RunContext context, IPeriodicTask task) {
        tasks.add(task);
        task.onStart(context);
    }

    public void remove(IPeriodicTask task) {
        tasks.remove(task);
    }

    public void clear() {

    }

    public void process(RunContext context) {
        for (IPeriodicTask task : tasks) {
            if(task.getAllowedRunContexts().contains(context)) {
                task.onLoop(context);
            }
        }
    }
}
