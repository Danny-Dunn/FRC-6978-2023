package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Hardware;

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
        if(calibrating) {
            if(Hardware.armSlideMotor.getSelectedSensorVelocity() < 250) {
                calibrating = false;
                SmartDashboard.putBoolean("ArmSlideCalibrating", calibrating);
                Hardware.armSlideMotor.setSelectedSensorPosition(Constants.Arm.slideStartingPosition);
            }
            return;
        }

        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.retractSlide)) {
            Hardware.armSlideMotor.set(ControlMode.Position, Constants.Arm.slideParkPosition);
        }

        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.midDeploySlide)) {
            Hardware.armSlideMotor.set(ControlMode.Position, Constants.Arm.slideMidPosition);
        }

        if(Hardware.operatorStick.getRawButtonPressed(Constants.OperatorControls.fullDeploySlide)) {
            Hardware.armSlideMotor.set(ControlMode.Position, Constants.Arm.slideMaxExtension);
        }
    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
