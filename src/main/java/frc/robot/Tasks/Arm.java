package frc.robot.Tasks;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Framework.JoystickHelpers;
import frc.robot.Framework.RunContext;
import frc.robot.Platform.Constants;
import frc.robot.Platform.Globals;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Constants.OperatorControls;

public class Arm {
    double liftTarget, cableTarget, slideTarget;
    boolean gripperState;

    public void onStart(RunContext context) {
        gripperState = Constants.Arm.gripperCloseDefaultState;
        Hardware.gripperSolenoid.set(gripperState);
    }

    public void onStop() {}

    public void onLoop(RunContext context) {
        SmartDashboard.putNumber("ArmLiftPosition", getLiftPosition());
        SmartDashboard.putNumber("ArmCablePosition", getCablePosition());
        SmartDashboard.putNumber("ArmSlidePosition", getSlidePosition());

        if(Hardware.operatorStick.getRawButtonReleased(OperatorControls.zeroArm)) {
            Hardware.armCableMotor.setSelectedSensorPosition(0);
            Hardware.armLiftMotor.setSelectedSensorPosition(0);
            Hardware.armSlideMotor.setSelectedSensorPosition(0);
        }

        if(Hardware.operatorStick.getRawButtonPressed(OperatorControls.toggleArmAutomation)) {
            Globals.armAutomation = !Globals.armAutomation;
        }

        SmartDashboard.putBoolean("Arm Automation", Globals.armAutomation);

        //Claw open/close
        if(Hardware.operatorStick.getRawButtonPressed(OperatorControls.toggleClaw)) {
            gripperState = !gripperState;
            Hardware.gripperSolenoid.set(gripperState);
        }

        //Gripper wheels
        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.slowWheelsIn)) {
            Hardware.gripperWheelsMotor.set(ControlMode.PercentOutput, Constants.Arm.wheelSlow);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.fastWheelsIn)) {
            Hardware.gripperWheelsMotor.set(ControlMode.PercentOutput, Constants.Arm.wheelSlow);
        }

        if(Hardware.driverStick.getRawButtonPressed(Constants.DriverControls.fastWheelsOut)) {
            Hardware.gripperWheelsMotor.set(ControlMode.PercentOutput, -Constants.Arm.wheelLaunch);
        }

        if(
            Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.fastWheelsIn) ||
            Hardware.driverStick.getRawButtonReleased(Constants.DriverControls.fastWheelsOut)
        ) {
            Hardware.gripperWheelsMotor.set(ControlMode.Disabled, 0);
        }

        //do manual stick and button inputs
        if(!Globals.armAutomation) {  
            //lift
            Hardware.armLiftMotor.set(
                ControlMode.PercentOutput, 
                JoystickHelpers.deadZone(
                    Hardware.operatorStick.getRawAxis(Constants.OperatorControls.liftAxis), 
                    0.1)
            );
            
            //cable
            Hardware.armCableMotor.set(
                ControlMode.PercentOutput,
                JoystickHelpers.deadZone(
                    Hardware.operatorStick.getRawAxis(Constants.OperatorControls.cableAxis),
                    0.1)
            );

            //slide
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

        setLiftPosition(Constants.Arm.liftPositionMap.get(Globals.requestedArmPosition));
        setSlidePosition(Constants.Arm.slidePositionMap.get(Globals.requestedArmPosition));
        setCablePosition(Constants.Arm.cablePositionMap.get(Globals.requestedArmPosition));
    }

    //Lift

    public void setLiftPosition(double position) {
        liftTarget = position;
        //Constraints
        double maximum = Constants.Arm.liftUpperPosition;
        double minimum = Constants.Arm.liftLowerPosition;

        //Constrain minimum to top when claw is inside bumper
        if(
            getCablePosition() < Constants.Arm.cableBumperClearancePosition && 
            getLiftPosition() > Constants.Arm.liftInsideBumperLower
        ) {
            minimum = 0;
            Globals.ArmConstraints.liftClawPlexiglassCrash = true;
        } else {
            Globals.ArmConstraints.liftClawPlexiglassCrash = false;
        }

        //Don't allow a ground crash when the claw is below the bumper and the lift is already low
        if (
            getCablePosition() < Constants.Arm.cableBumperClearancePosition && 
            getLiftPosition() < Constants.Arm.liftInsideBumperLower
        ) {
            maximum = Constants.Arm.liftInsideBumperLower - 5000;
            Globals.ArmConstraints.liftClawTiltCrash = true;
        } else {
            Globals.ArmConstraints.liftClawTiltCrash = false;
        }

        position = (position > maximum)? maximum:position;
        position = (position < minimum)? minimum:position;

        SmartDashboard.putNumber("Lift min position", minimum);
        SmartDashboard.putNumber("Lift maximum position", maximum);

        Hardware.armLiftMotor.set(ControlMode.Position, position);
    }

    public double getLiftPosition() {
        return Hardware.armLiftMotor.getSelectedSensorPosition();
    }

    public double getLiftTarget() {
        return liftTarget;
    }

    //Cable

    public void setCablePosition(double position) {
        double minimum = Constants.Arm.cableStartingPosition;
        double maximum = Constants.Arm.cableMaxPosition;

        //constrain above bumper clearance when lift needs to drop
        if(
            (//when lift is above bottom of zone and wants to come down
                getLiftPosition() > Constants.Arm.liftInsideBumperLower &&
                getLiftTarget() < Constants.Arm.liftInsideBumperLower
            ) || ( //when lift is below top of zone and wants to go up
                getLiftPosition() < Constants.Arm.liftInsideBumperUpper &&
                getLiftTarget() > Constants.Arm.liftInsideBumperUpper
            ) || (//when lift wants to target zone
                getLiftTarget() > Constants.Arm.liftInsideBumperLower &&
                getLiftTarget() < Constants.Arm.liftInsideBumperUpper
            ) || (//when lift is inside zone
                getLiftPosition() > Constants.Arm.liftInsideBumperLower &&
                getLiftPosition() < Constants.Arm.liftInsideBumperUpper
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
        SmartDashboard.putNumber("Cable target position", position);


        Hardware.armCableMotor.set(ControlMode.Position, position);
    }

    public double getCablePosition() {
        return Hardware.armCableMotor.getSelectedSensorPosition();
    }

    public double getCableTarget() {
        return cableTarget;
    }

    //Slide

    public void setSlidePosition(double position) {
        slideTarget = position;

        double minimum = Constants.Arm.slideParkPosition;
        double maximum = Constants.Arm.slideMaxExtension;

        if( //inside bumper
            (
                getLiftPosition() > Constants.Arm.liftInsideBumperLower &&
                getCablePosition() < Constants.Arm.cableBumperClearancePosition
            ) || (
                Globals.ArmConstraints.cableBumperClearance
            )
        ) {
            maximum = 0;
        }

        position = (position > maximum)? maximum:position;
        position = (position < minimum)? minimum:position;

        Hardware.armSlideMotor.set(ControlMode.Position, position);
    }

    public double getSlidePosition() {
        return Hardware.armSlideMotor.getSelectedSensorPosition();
    }

    public double getSlideTarget() {
        return slideTarget;
    }
}
