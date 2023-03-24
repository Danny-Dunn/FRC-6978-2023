package frc.robot.Platform;

import frc.robot.Tasks.*;
import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    public static DifferentialDrive differentialDrive = new DifferentialDrive();
    public static Arm arm = new Arm();

    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)differentialDrive,
        (IPeriodicTask)arm
    };
}
