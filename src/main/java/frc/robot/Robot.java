package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Schedulers;
import frc.robot.Platform.Tasks;

public class Robot extends TimedRobot{
    @Override
    public void robotInit() {
        Hardware.configureHardware();
        
    }

    @Override
    public void teleopInit() {
        Schedulers.teleopScheduler.clear();
        for (IPeriodicTask task : Tasks.teleopTasks) {
            Schedulers.teleopScheduler.add(RunContext.teleoperated, task);
        }
    }

    @Override
    public void teleopPeriodic() {
        Schedulers.teleopScheduler.process(RunContext.teleoperated);
    }
    
    @Override
    public void disabledInit() {
        Schedulers.teleopScheduler.clear();
    }
}