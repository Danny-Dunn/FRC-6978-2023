package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.PIDController;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Tasks;

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

        //FIXME: what was this meant to do???
        if(Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.balance)) {
            //useStick();
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
        Tasks.telemetry.pushEvent("DifferentialDrive.EnterStick");
        setBrake(false);
        mode = DriveMode.stick;
    }

    void onStickDrive() {
        double x;
        double y;

        x = Hardware.driverStick.getRawAxis(Constants.DriverControls.steeringAxis);
        y = (Hardware.driverStick.getRawAxis(Constants.DriverControls.forwardAxis) + 1 )/2 - 
            (Hardware.driverStick.getRawAxis(Constants.DriverControls.reverseAxis) + 1 )/2;

        x = (Math.abs(x) > Constants.Drive.deadZone)? 
            ((x > 0)? 
                ((x-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((x+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        y = (Math.abs(y) > Constants.Drive.deadZone)? 
            ((y > 0)? 
                ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) :
                ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone))
            ) 
            : 0;

        x *= Constants.Drive.xMultiplier;
        y *= Constants.Drive.yMultiplier;

        Tasks.telemetry.pushDouble("DriveX", x);
        Tasks.telemetry.pushDouble("DriveY", y);

        setDrives(x, y);        
    }

    void onBalance() {
        setDrives(
            balancingYawController.process(Hardware.navX.getAngle()), 
            balancingPitchController.process(Hardware.navX.getPitch())
            );
    }

    public void balance() {
        Tasks.telemetry.pushEvent("DifferentialDrive.EnterBalance");

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
        
        double leftSpeed = (y + x)*Constants.Drive.maxVelocity;
        double rightSpeed = (y - x)*Constants.Drive.maxVelocity;

        if(x == 0 && y == 0) {
            Hardware.leftDrive1.set(ControlMode.Disabled, 0);
            Hardware.rightDrive1.set(ControlMode.Disabled, 0);
        } else {
            Hardware.leftDrive1.set(ControlMode.Velocity, leftSpeed);
            Hardware.rightDrive1.set(ControlMode.Velocity, rightSpeed);
            Tasks.telemetry.pushDouble("DifferentialDrive.leftVelocityTarget", leftSpeed);
            Tasks.telemetry.pushDouble("DifferentialDrive.rightVelocityTarget", rightSpeed);
        }
    }

    void setBrake(boolean brake) {
        Tasks.telemetry.pushEvent("DifferentialDrive.SetBrakeMode");
        Tasks.telemetry.pushBoolean("DifferentialDrive.BrakeMode", brake);
        NeutralMode neutralMode = brake? NeutralMode.Brake : NeutralMode.Coast; 
        Hardware.leftDrive1.setNeutralMode(neutralMode);
        Hardware.leftDrive2.setNeutralMode(neutralMode);
        Hardware.rightDrive1.setNeutralMode(neutralMode);
        Hardware.rightDrive2.setNeutralMode(neutralMode);
    }

    public void setGearShift(boolean highGear) {
        Tasks.telemetry.pushEvent("DifferentialDrive.ShiftGears");
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
