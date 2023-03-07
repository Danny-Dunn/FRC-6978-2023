package frc.robot.Platform;

import java.util.List;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;

public class Scheduler {
    public static List<IPeriodicTask> tasks;

    public static void add(RunContext context, IPeriodicTask task) {
        tasks.add(task);
        task.onStart(context);
    }

    public static void remove(IPeriodicTask task) {
        tasks.remove(task);
    }

    public static void process(RunContext context) {
        for (IPeriodicTask task : tasks) {
            if(task.getAllowedRunContexts().contains(context)) {
                task.onLoop(context);
            }
        }
    }
}
