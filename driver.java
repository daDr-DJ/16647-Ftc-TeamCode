package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
//@Disabled

public class driver extends LinearOpMode {

    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;
    DcMotor A;
    DcMotor D;
    Servo G;

    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.dcMotor.get("left front");
        RF = hardwareMap.dcMotor.get("right front");
        LB = hardwareMap.dcMotor.get("left back");
        RB = hardwareMap.dcMotor.get("right back");
        A = hardwareMap.dcMotor.get("arm");
        D = hardwareMap.dcMotor.get("duck");
        G = hardwareMap.servo.get("grabber");

        //S.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            //driving
            double l = gamepad1.left_stick_y;
            double r = gamepad1.right_stick_y;
            double p = 0.5;
            //---------------------------------------------------

            if (gamepad1.left_bumper) {
                LF.setPower(l);
                LB.setPower(l);
                RF.setPower(r);
                RB.setPower(r);
            } else {
                LF.setPower(l * p);
                LB.setPower(l * p);
                RF.setPower(r * p);
                RB.setPower(r * p);
            }
            //---------------------------------------------------------

            //arm
            /*
            boolean active = false;
            if (gamepad2.a && active==false) {
                active = true;
                A.setPower(0.3);
                sleep(1100);
                A.setPower(0);
                active = false;
            }
            if (gamepad2.b && active==false) {
                active = true;
                A.setPower(-0.3);
                sleep(1000);
                A.setPower(0);
                active = false;
            }
            */
            if (gamepad2.right_trigger>0.05) {
                A.setPower(0.3);
            } else
            if (gamepad2.left_trigger>0.05) {
                A.setPower(-0.3);
            } else {
                A.setPower(0);
            }
            //-----------------------------------------------------------

            //grabber
            if (gamepad2.x) {
                G.setPosition(0);
            }
            if (gamepad2.y) {
                G.setPosition(1);
            }
            //---------------------------------------------------------

            //duckwheel
            if (gamepad2.dpad_right) {
                D.setPower(-0.13);
            }
            if (gamepad2.dpad_left) {
                D.setPower(0.13);
            }
            if (gamepad2.dpad_down) {
                D.setPower(0);
            }
            //-----------------------------------------------------




        }
    }
}
