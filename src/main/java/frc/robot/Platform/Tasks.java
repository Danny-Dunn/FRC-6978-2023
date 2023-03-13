package frc.robot.Platform;

import frc.robot.Tasks.*;
import frc.robot.Framework.IPeriodicTask;

public class Tasks {
    public static DifferentialDrive differentialDrive = new DifferentialDrive();
    public static ArmSlide armSlide = new ArmSlide();
    public static ArmCable armCable = new ArmCable();
    public static ArmLift  armLift  = new ArmLift();
    public static ArmButtons armButtons = new ArmButtons();

    public static IPeriodicTask[] teleopTasks = {
        (IPeriodicTask)differentialDrive,
        (IPeriodicTask)armSlide,
        (IPeriodicTask)armCable,
        (IPeriodicTask)armLift,
        (IPeriodicTask)armButtons
    };
}
