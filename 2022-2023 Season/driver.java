package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@TeleOp
//@Disabled

public class driver extends LinearOpMode {

    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;
    DcMotor A;
    Servo GA;
    Servo GB;

    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.dcMotor.get("left front");
        RF = hardwareMap.dcMotor.get("right front");
        LB = hardwareMap.dcMotor.get("left back");
        RB = hardwareMap.dcMotor.get("right back");
        A = hardwareMap.dcMotor.get("arm");
        GA = hardwareMap.servo.get("wheel1");
        GB = hardwareMap.servo.get("wheel2");


        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            //driving
            double ly = gamepad1.left_stick_y;
            double ry = gamepad1.right_stick_y;
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double p = 0.5;
            //---------------------------------------------------

            if (gamepad1.left_bumper) {
                LF.setPower((ly)-(lx));
                LB.setPower((ly)+(lx));
                RF.setPower((ry)+(rx));
                RB.setPower((ry)-(rx));
            } else {
                LF.setPower((ly)*p-(lx)*p);
                LB.setPower((ly)*p+(lx)*p);
                RF.setPower((ry)*p+(rx)*p);
                RB.setPower((ry)*p-(rx)*p);
            }


            //---------------------------------------------------------

            if (gamepad2.right_trigger>0.1) {
                A.setPower(0.3);
            } else
            if (gamepad2.left_trigger>0.1) {
                A.setPower(-0.3);
            } else {
                A.setPower(0);
            }
            //-----------------------------------------------------------

            //grabber
            if (gamepad2.right_bumper) {
                GA.setPosition(-1);
                GB.setPosition(-1);
            }
            if (gamepad2.left_bumper) {
                GA.setPosition(1);
                GB.setPosition(1);
            }
            //---------------------------------------------------------

            

        }
    }
}
