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

public class ArmSlide implements IPeriodicTask {
    double target;
    boolean calibrating;

    public void onStart(RunContext context) {
        Hardware.armSlideMotor.set(ControlMode.PercentOutput, -0.35);
        calibrating = true;
        SmartDashboard.putBoolean("ArmSlideCalibrating", calibrating);
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        SmartDashboard.putNumber("ArmSlidePosition", Hardware.armSlideMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber("SlideCurrent", Hardware.armSlideMotor.getStatorCurrent());

        if(calibrating) {
            if(Hardware.armSlideMotor.getSelectedSensorVelocity() < 250) {
                calibrating = false;
                SmartDashboard.putBoolean("ArmSlideCalibrating", calibrating);
                Hardware.armSlideMotor.setSelectedSensorPosition(Constants.Arm.slideStartingPosition);
            }
            return;
        }

        if(!Globals.armAutomation) {
            if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.slideOut)) {
                Hardware.armSlideMotor.set(ControlMode.PercentOutput, 0.4);
            } else if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.slideIn)) {
                Hardware.armSlideMotor.set(ControlMode.PercentOutput, -0.4);
            } else if(Hardware.operatorStick.getRawButton(Constants.OperatorControls.zeroArm)) {
                Hardware.armSlideMotor.set(ControlMode.PercentOutput, -0.1);
            } else {
                Hardware.armSlideMotor.set(ControlMode.Disabled, 0);
            }
            return;
        }

        switch (Globals.requestedArmPosition) {
            case park:
                setPosition(0);
                break;
            case partialPark:
                setPosition(0);
                break;
            case groundPickup:
                setPosition(0);
                break;
            case humanPickup:
                setPosition(0);
                break;
            case cubeMid:
                setPosition(0);
                break;
            case coneMid:
                setPosition(10051);
                break;
            case cubeHigh:
                setPosition(17000);
                break;
            case coneHigh:
                setPosition(19900);
                break;
        }
    }

    public void setPosition(double position) {
        target =  position;

        double minimum = Constants.Arm.slideParkPosition;
        double maximum = Constants.Arm.slideMaxExtension;

        if( //inside bumper
            (
                Tasks.armLift.getPosition() > Constants.Arm.liftInsideBumperLower &&
                Tasks.armCable.getPosition() < Constants.Arm.cableBumperClearancePosition
            ) || (
                Globals.ArmConstraints.cableBumperClearance
            )
        ) {
            maximum = 0;
        }

        position = (position > maximum)? maximum:position;
        position = (position < minimum)? minimum:position;

        Hardware.armSlideMotor.set(ControlMode.Position, Constants.Arm.slideParkPosition);
    }

    public double getTarget() {
        return target;
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
