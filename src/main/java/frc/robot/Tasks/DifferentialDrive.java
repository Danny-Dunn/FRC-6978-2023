package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;

public class DifferentialDrive implements IPeriodicTask{
    int mode;
    PIDController balancingYawController = new PIDController(
        Constants.Drive.balancer_yaw_kP,
        Constants.Drive.balancer_yaw_kI,
        Constants.Drive.balancer_yaw_kD
    );
    PIDController balancingPitchController = new PIDController(
        Constants.Drive.balancer_pitch_kP,
        Constants.Drive.balancer_pitch_kI,
        Constants.Drive.balancer_pitch_kD
    );

    boolean shiftState;
    
    public void onStart(RunContext context) {
        mode = 0;
        shiftState = Constants.Drive.gearShiftDefaultState;
        Hardware.driveGearShiftSolenoid.set(shiftState);
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.gearShift)) {
            shiftState = !shiftState;
            Hardware.driveGearShiftSolenoid.set(shiftState);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.balance)) {
            balancingYawController.init();
            balancingPitchController.init();
            //target perfectly flat
            balancingPitchController.set(0);
            ///target heading hold
            balancingYawController.set(Hardware.navX.getAngle());
            mode = 1;

            //set brake mode 
            setBrake(true);
        }

        if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.balance)) {
            mode = 0;
            setBrake(false);
        }
        
        switch (mode) {
            case 0:
                stickDrive();
                break;
            case 1:
                balance();
                break;
            default:
                break;
        }
    }

    void stickDrive() {
        double x;
        double y;

        //TODO: TOP!! fix drive inversion
        x = Hardware.driverStick.getRawAxis(0);
        y = (Hardware.driverStick.getRawAxis(4) + 1 )/2 - (Hardware.driverStick.getRawAxis(3) + 1 )/2 ;

        //x = x * x *x; 
       

        x = (Math.abs(x) < Constants.Drive.deadZone)? 0 : x;
        x = (x > Constants.Drive.deadZone)? ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : x;
        x = (x < -Constants.Drive.deadZone)? ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : x;

        y = (Math.abs(y) < Constants.Drive.deadZone)? 0 : y;
        y = (y > Constants.Drive.deadZone)? ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;
        y = (y < -Constants.Drive.deadZone)? ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;

        x *= Constants.Drive.xMultiplier;
        y *= Constants.Drive.yMultiplier;

        SmartDashboard.putNumber("DriveX", x);
        SmartDashboard.putNumber("DriveY", y);

        setDrives(x, y);        
    }

    void balance() {
        setDrives(
            balancingYawController.process(Hardware.navX.getAngle()), 
            balancingPitchController.process(Hardware.navX.getPitch())
            );
    }

    void setDrives(double x, double y) {
        if(y > Constants.Drive.maxY){
            y = Constants.Drive.maxY;
        }else if(y < -Constants.Drive.maxY){
            y = -Constants.Drive.maxY;
        }

        if(x > Constants.Drive.maxY){
            x = Constants.Drive.maxY;
        }else if(x < -Constants.Drive.maxY){
            x = -Constants.Drive.maxY;
        }
        
        if(x == 0 && y == 0) {
            Hardware.leftDrive1.set(ControlMode.Disabled, 0);
            Hardware.rightDrive1.set(ControlMode.Disabled, 0);
        } else {
            Hardware.leftDrive1.set(ControlMode.Velocity, (y + x)*Constants.Drive.maxVelocity );
            Hardware.rightDrive1.set(ControlMode.Velocity, (y - x)*Constants.Drive.maxVelocity);
        }
    }

    void setBrake(boolean brake) {
        NeutralMode neutralMode = brake? NeutralMode.Brake : NeutralMode.Coast; 
        Hardware.leftDrive1.setNeutralMode(neutralMode);
        Hardware.leftDrive2.setNeutralMode(neutralMode);
        Hardware.rightDrive1.setNeutralMode(neutralMode);
        Hardware.rightDrive2.setNeutralMode(neutralMode);
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
