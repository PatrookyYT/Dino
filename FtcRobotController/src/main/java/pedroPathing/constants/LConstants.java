package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .001989436789;
        ThreeWheelIMUConstants.strafeTicksToInches = .001989436789;
        ThreeWheelIMUConstants.turnTicksToInches = .001989436789;
        ThreeWheelIMUConstants.leftY = 3;
        ThreeWheelIMUConstants.rightY = -3;
        ThreeWheelIMUConstants.strafeX = -7.5;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);    }
}




