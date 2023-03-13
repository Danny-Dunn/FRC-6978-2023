package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Globals;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Tasks;

public class ArmLift implements IPeriodicTask{
    double target;

    public void onStart(RunContext context) {
        Hardware.armLiftMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        if(!Globals.armAutomation) {

        }

        switch (Globals.requestedArmPosition) {
            case park:
                setPosition(0);
                break;
            case partialPark:
                setPosition(0);
                break;
            case groundPickup:
                setPosition(-640000);
                break;
            case humanPickup:
                setPosition(-370000);
                break;
            case cubeMid:
                setPosition(-531000);
                break;
            case coneMid:
                setPosition(-531000);
                break;
            case cubeHigh:
                setPosition(-358000);
                break;
            case coneHigh:
                setPosition(-358000);
                break;
        }
    }

    public void setPosition(double position) {
        target = position;
        //Constraints
        double maximum = Constants.Arm.liftUpperPosition;
        double minimum = Constants.Arm.liftLowerPosition;

        //Constrain minimum to top when claw is inside bumper
        if(
            Tasks.armCable.getPosition() < Constants.Arm.cableBumperClearancePosition && 
            getPosition() > Constants.Arm.liftInsideBumperLower
        ) {
            minimum = 0;
        }

        //Don't allow a ground crash when the claw is below the bumper and the lift is already low
        if (
            Tasks.armCable.getPosition() < Constants.Arm.cableBumperClearancePosition && 
            getPosition() < Constants.Arm.liftInsideBumperLower
        ) {
            maximum = Constants.Arm.liftInsideBumperLower - 5000;
        }

        position = (position > maximum)? maximum:position;
        position = (position < minimum)? minimum:position;

        Hardware.armLiftMotor.set(ControlMode.Position, position);
    }

    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return Hardware.armLiftMotor.getSelectedSensorPosition();
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
