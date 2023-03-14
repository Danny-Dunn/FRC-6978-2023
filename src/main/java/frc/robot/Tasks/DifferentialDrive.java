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
    enum DriveMode {
        stick,
        balance,
        heading,
        distance
    };
    DriveMode mode;

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

    boolean gearShiftState;
    
    public DifferentialDrive() {
    }

    public void onStart(RunContext context) {
        gearShiftState = Constants.Drive.gearShiftDefaultState;
        Hardware.driveGearShiftSolenoid.set(gearShiftState);
        if(context == RunContext.teleoperated) {
            useStick();
        }
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.gearShift)) {
            setGearShift(!gearShiftState);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.balance)) {
            balance();
        }

        if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.balance)) {
            useStick();
        }
        
        switch (mode) {
            case stick:
                onStickDrive();
                break;
            case balance:
                onBalance();
                break;
            default:
                break;
        }
    }

    public void useStick() {
        setBrake(false);
        mode = DriveMode.stick;
    }

    void onStickDrive() {
        double x;
        double y;

        //TODO: TOP!! fix drive inversion
        x = Hardware.driverStick.getRawAxis(Constants.DriverControls.steeringAxis);
        y = (Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.driverStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;

        x = (Math.abs(x) < Constants.Drive.deadZone)? 0 : x;
        x = (x > Constants.Drive.deadZone)? 
            ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : x;

        x = (x < -Constants.Drive.deadZone)? 
            ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : x;

        y = (Math.abs(y) < Constants.Drive.deadZone)? 0 : y;
        y = (y > Constants.Drive.deadZone)? 
            ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;

        y = (y < -Constants.Drive.deadZone)? 
            ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;

        x *= Constants.Drive.xMultiplier;
        y *= Constants.Drive.yMultiplier;

        SmartDashboard.putNumber("DriveX", x);
        SmartDashboard.putNumber("DriveY", y);

        setDrives(x, y);        
    }

    void onBalance() {
        setDrives(
            balancingYawController.process(Hardware.navX.getAngle()), 
            balancingPitchController.process(Hardware.navX.getPitch())
            );
    }

    public void balance() {
        
        balancingYawController.init();
        balancingPitchController.init();
        //target perfectly flat
        balancingPitchController.set(0);
        ///target heading hold
        balancingYawController.set(Hardware.navX.getAngle());
        mode = DriveMode.balance;

        //set brake mode 
        setBrake(true);
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

    public void setGearShift(boolean highGear) {
        gearShiftState = highGear;
        Hardware.driveGearShiftSolenoid.set(highGear);
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
