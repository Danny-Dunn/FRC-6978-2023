package frc.robot.Tasks;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.IPeriodicTask;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.ArmPosition;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Constants.OperatorControls;
import frc.robot.Platform.Globals;
import frc.robot.Platform.Hardware;

public class ArmButtons implements IPeriodicTask {
    boolean gripperState;
    
    public void onStart(RunContext context){ 
        gripperState = Constants.Arm.gripperCloseDefaultState;
        Hardware.gripperSolenoid.set(gripperState);
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        if(Hardware.operatorStick.getRawButtonReleased(OperatorControls.zeroArm)) {
            Hardware.armCableMotor.setSelectedSensorPosition(0);
            Hardware.armLiftMotor.setSelectedSensorPosition(0);
            Hardware.armSlideMotor.setSelectedSensorPosition(0);
        }

        if(Hardware.operatorStick.getRawButtonPressed(OperatorControls.toggleArmAutomation)) {
            Globals.armAutomation = !Globals.armAutomation;
        }

        SmartDashboard.putBoolean("Arm Automation", Globals.armAutomation);

        if(Hardware.operatorStick.getRawButtonPressed(OperatorControls.toggleClaw)) {
            gripperState = !gripperState;
            Hardware.gripperSolenoid.set(gripperState);
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.park)){
            Globals.requestedArmPosition = ArmPosition.park;
        }

        if(Hardware.operatorStick.getPOV() == OperatorControls.partialParkPOV){
            Globals.requestedArmPosition = ArmPosition.park;
        }
        
        if(Hardware.operatorStick.getRawButton(OperatorControls.groundPickup)){
            Globals.requestedArmPosition = ArmPosition.groundPickup;
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.humanPickup)){
            Globals.requestedArmPosition = ArmPosition.humanPickup;
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.cubeMid)){
            Globals.requestedArmPosition = ArmPosition.cubeMid;
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.coneMid)){
            Globals.requestedArmPosition = ArmPosition.coneMid;
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.cubeHigh)){
            Globals.requestedArmPosition = ArmPosition.cubeHigh;
        }

        if(Hardware.operatorStick.getRawButton(OperatorControls.coneHigh)){
            Globals.requestedArmPosition = ArmPosition.coneHigh;
        }

        SmartDashboard.putString("Requested Arm Position", Globals.requestedArmPosition.toString());
        SmartDashboard.putBoolean("constrainLiftClawPlexiglassCrash", Globals.ArmConstraints.liftClawPlexiglassCrash);
        SmartDashboard.putBoolean("constrainLiftClawTiltCrash", Globals.ArmConstraints.liftClawTiltCrash);
        SmartDashboard.putBoolean("constrainCableBumperClearance", Globals.ArmConstraints.cableBumperClearance);

    }

    public List<RunContext> getAllowedRunContexts() { 
        return new ArrayList<RunContext>(){{
            add(RunContext.autonomous);
            add(RunContext.teleoperated);
        }};
    }
}
