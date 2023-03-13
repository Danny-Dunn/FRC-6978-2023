package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Globals;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Tasks;

public class ArmCable implements IPeriodicTask {
    double target;

    public void onStart(RunContext context) {}

    public void onStop() {}

    public void onLoop(RunContext context) {
        if(!Globals.armAutomation) {
            double y = Hardware.operatorStick.getRawAxis(Constants.OperatorControls.cableAxis);

            y = (Math.abs(y) < Constants.Drive.deadZone)? 0 : y;
            y = (y > Constants.Drive.deadZone)? ((y-Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;
            y = (y < -Constants.Drive.deadZone)? ((y+Constants.Drive.deadZone)/(1-Constants.Drive.deadZone)) : y;
            
            Hardware.armCableMotor.set(ControlMode.PercentOutput, y);
            return;
        }
        switch (Globals.requestedArmPosition) {
            case park:
                setPosition(0);
                break;
            case partialPark:
                setPosition(37000);
                break;
            case groundPickup:
                setPosition(-76000);
                break;
            case humanPickup:
                setPosition(362000);
                break;
            case cubeMid:
                setPosition(226174);
                break;
            case coneMid:
                setPosition(336000);
                break;
            case cubeHigh:
                setPosition(370000);
                break;
            case coneHigh:
                setPosition(402000);
                break;
        }
    }

    public void setPosition(double position) {
        target = position;
        double minimum = Constants.Arm.cableStartingPosition;
        double maximum = Constants.Arm.cableMaxPosition;

        //constrain above bumper clearance when lift needs to drop
        if(
            (//when lift is above bottom of zone and wants to come down
                Tasks.armLift.getPosition() > Constants.Arm.liftInsideBumperLower &&
                Tasks.armLift.getTarget() < Constants.Arm.liftInsideBumperLower
            ) || ( //when lift is below top of zone and wants to go up
                Tasks.armLift.getPosition() < Constants.Arm.liftInsideBumperUpper &&
                Tasks.armLift.getTarget() > Constants.Arm.liftInsideBumperUpper
            ) || (//when lift wants to target zone
                Tasks.armLift.getTarget() > Constants.Arm.liftInsideBumperLower &&
                Tasks.armLift.getTarget() < Constants.Arm.liftInsideBumperUpper
            ) || (//when lift is inside zone
                Tasks.armLift.getPosition() > Constants.Arm.liftInsideBumperLower &&
                Tasks.armLift.getPosition() < Constants.Arm.liftInsideBumperUpper
            )
        ) {
            minimum = Constants.Arm.cableBumperClearancePosition + 5000;
            Globals.ArmConstraints.cableBumperClearance = true;
        } else {
            Globals.ArmConstraints.cableBumperClearance = false;
        }

        position = (position > maximum)? maximum:position;
        position = (position < minimum)? minimum:position;

        SmartDashboard.putNumber("Cable min position", minimum);
        SmartDashboard.putNumber("Cable maximum position", maximum);

        Hardware.armCableMotor.set(ControlMode.Position, Constants.Arm.slideParkPosition);
    }

    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return Hardware.armSlideMotor.getSelectedSensorPosition();
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
