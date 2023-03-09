package frc.robot.Platform;

import frc.robot.Tasks.DifferentialDrive;
import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    static DifferentialDrive differentialDrive = new DifferentialDrive();

    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)differentialDrive
    };
}
