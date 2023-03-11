package frc.robot.Platform;

import frc.robot.Tasks.*;
import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    static DifferentialDrive differentialDrive = new DifferentialDrive();
    static ArmSlide armSlide = new ArmSlide();

    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)differentialDrive,
        (IPeriodicTask)armSlide
    };
}
