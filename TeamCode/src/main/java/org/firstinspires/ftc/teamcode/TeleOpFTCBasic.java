package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpFTCBasic (Blocks to Java)")
public class TeleOpFTCBasic extends LinearOpMode {

    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double D_Speed;
        float vertical_2;
        float vertical;
        float horizontal;
        float pivot;

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        waitForStart();
        if (opModeIsActive()) {
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
            while (opModeIsActive()) {
                telemetry.update();
                //
                // Used for stopping motor and servo movement
                //
                D_Speed = 0.5;
                //
                // Used for arm movement
                //
                vertical_2 = gamepad2.right_stick_y;
                //
                // Used for wheel movement
                //
                vertical = -gamepad1.left_stick_x;
                //
                // Used for wheel movement
                //
                horizontal = gamepad1.left_stick_y;
                //
                // Used for wheel movement
                //
                pivot = gamepad1.left_stick_x;
                //
                // Gamepad 1 commands start
                //
                // Used for Mecanum wheel movement
                //
                BackLeft.setPower(vertical - horizontal);
                BackRight.setPower(vertical + horizontal);
                FrontLeft.setPower(vertical + horizontal);
                FrontRight.setPower(vertical - horizontal);
                if (gamepad1.dpad_left) {
                    // Turns the robot left (Hopefully)
                    //
                    BackLeft.setPower(-D_Speed);
                    BackRight.setPower(-D_Speed);
                    FrontLeft.setPower(D_Speed);
                    FrontRight.setPower(D_Speed);
                    //
                } else if (gamepad1.dpad_up) {
                    // Makes the robot go forward (Hopefully)
                    //
                    BackLeft.setPower(D_Speed);
                    BackRight.setPower(-D_Speed);
                    FrontLeft.setPower(-D_Speed);
                    FrontRight.setPower(D_Speed);
                    //
                } else if (gamepad1.dpad_down) {
                    // Makes the rbot go backwards (Hopefully)
                    //
                    BackLeft.setPower(-D_Speed);
                    BackRight.setPower(D_Speed);
                    FrontLeft.setPower(D_Speed);
                    FrontRight.setPower(-D_Speed);
                    //
                } else if (gamepad1.y) {
                } else if (gamepad1.b) {
                } else if (gamepad1.left_bumper) {
                } else if (gamepad1.dpad_right) {
                    // Turns the robot right (Hopefully)
                    //
                    BackLeft.setPower(D_Speed);
                    BackRight.setPower(D_Speed);
                    FrontLeft.setPower(-D_Speed);
                    FrontRight.setPower(-D_Speed);
                    //
                } else if (gamepad1.x) {
                } else if (gamepad1.a) {
                } else if (gamepad2.y) {
                } else if (gamepad1.left_bumper) {
                } else if (vertical_2 < 0) {
                } else if (false) {
                } else if (vertical_2 > 0) {
                    // Gamepad 2 commands end
                } else if (gamepad2.x) {
                } else if (gamepad2.b) {
                } else if (gamepad2.dpad_up) {
                } else if (gamepad2.dpad_down) {
                } else {
                    // Stops all movement
                    //
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    FrontLeft.setPower(0);
                    FrontRight.setPower(0);
                }
            }
        }
    }
}
