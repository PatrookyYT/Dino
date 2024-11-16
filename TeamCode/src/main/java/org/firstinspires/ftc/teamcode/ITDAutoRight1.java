package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ITD AutoRight1 (High-bar)")
public class ITDAutoRight1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public ServoController ControlHub_ServoController;
    private CRServo ClawArmServo;
    private DcMotor FrontArmMotor;

    @Override
    public void runOpMode() {

        boolean Running = true;
        boolean testMode = true; // Shows Telemetry data for Functions.drive and Functions.turn

        // Report that op mode has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //***VERY IMPORTANT**
        //Replace the device name (ex frontLeft) with the NAME OF THE
        //MOTORS DEFINED IN THE DRIVER HUB
        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        FrontArmMotor = hardwareMap.get(DcMotor.class, "FrontArmMotor");

        ClawArmServo = hardwareMap.get(CRServo.class, "ClawArmServo");

        FrontArmMotor.setDirection(DcMotor.Direction.REVERSE);

        FrontArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Disable pwm
        ControlHub_ServoController.pwmDisable();

        waitForStart();
        runtime.reset();

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////// Driving starts here//////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // 10in = 572

        if(Running)
        {
            // Drive to pixel

            ClawArmServo.setPower(1);

            //Fix Strafe
            Functions.drive(this, hardwareMap, telemetry, 8, -8, 0.45, -8, 8, testMode);
            Functions.pause(0.25);

            Functions.drive(this, hardwareMap, telemetry, -7, -7, 0.45, -7, -7, testMode);
            Functions.pause(0.25);

            Functions.turn(this, hardwareMap, telemetry, "Left", 0.45, testMode);
            Functions.pause(0.25);
//10.78
            Functions.drive(this, hardwareMap, telemetry, 11, 11, 0.45, 11, 11, testMode);
            Functions.pause(0.25);

            Functions.frontArmMove(this, hardwareMap, telemetry, 12, 0.25, "Front", testMode);
            Functions.pause(0.25);

            ClawArmServo.setPower(0);
            Functions.drive(this, hardwareMap, telemetry, -2.5, -2.5, 0.2, -2.5, -2.5, testMode);
            Functions.pause(0.25);

            Functions.drive(this, hardwareMap, telemetry, -11, -11, 0.45, -11, -11, testMode);
            Functions.pause(0.25);

            Functions.drive(this, hardwareMap, telemetry, -53, 53, 0.45, 53, -53, testMode);
            Functions.pause(0.25);
        }
    }

    //Very complicated code to make it so that when it is showing
    //seconds it has gone (directionTime) will be a one decimal number,
    //ex 0.2. Add 0's to both tens to increase the number of decimal
    //places shown.
    private String formatSeconds(double inputSeconds){
        double fixedValue = Math.floor(inputSeconds * 10) / 10;
        return String.valueOf(fixedValue);
    }
}