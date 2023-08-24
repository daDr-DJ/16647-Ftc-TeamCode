package teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import java.util.Arrays;
import java.util.List;

@TeleOp

public class driver_copy extends LinearOpMode {
    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;

    DcMotor SL,SR;
    List<DcMotor> SD;

    Servo AL;
    Servo AR;


    Servo CL;
    Servo CR;
    double open = 0.53;
    double closed = 0.4589;
    double LiftStartTime = 0;
    double ReturnStartTime = 0;



    int currentpos;

    //slide pos
    int reset = 0;
    int ground = -1;
    int low = -5;
    int mid = -10;

    int S_pos = reset;


    // other constants
    double p = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        //wheels
        LF = hardwareMap.dcMotor.get("left front");
        RF = hardwareMap.dcMotor.get("right front");
        LB = hardwareMap.dcMotor.get("left back");
        RB = hardwareMap.dcMotor.get("right back");

        //linear slides
        SL = hardwareMap.dcMotor.get("slide left");
        SR = hardwareMap.dcMotor.get("slide right");

        //Arm
        AL = hardwareMap.servo.get("arm left");
        AR = hardwareMap.servo.get("arm right");
        AR.resetDeviceConfigurationForOpMode();
        AL.resetDeviceConfigurationForOpMode();

        //claw
        CL = hardwareMap.servo.get("claw left");
        CR = hardwareMap.servo.get("claw right");
        //CR.resetDeviceConfigurationForOpMode();
        CL.resetDeviceConfigurationForOpMode();


        //Motor reverse
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo reverse
        AR.setDirection(Servo.Direction.REVERSE);



        SD = Arrays.asList(SL,SR);//SL,SR);


        waitForStart();

        while (opModeIsActive()) {

            Mechnum_Drive();

           // telemetry.update();

        }
    }
    public void Mechnum_Drive(){
        //input values
        double ly = gamepad1.left_stick_y;
        double ry = gamepad1.right_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        //Forward & Back
        if (gamepad1.left_bumper) {
            LF.setPower(((ly) - (lx)));
            LB.setPower(((ly) + (lx)));
            RF.setPower((ry) + (rx));
            RB.setPower((ry) - (rx));

            //Straif Left & Right
        } else {
            LF.setPower(((ly) * p - (lx) * p));
            LB.setPower(((ly) * p + (lx) * p));
            RF.setPower((ry) * p + (rx) * p);
            RB.setPower((ry) * p - (rx) * p);
        }


        Operations();

    }
    public void Operations() {

        //Claw
        if (gamepad1.left_bumper) {
            CL.setPosition(open);
            telemetry.addData("Keypad", "Claw = Open");

        } else if (gamepad1.right_bumper) {
            CL.setPosition(closed);
            telemetry.addData("Keypad", "Claw = Closed");
        }

        //Arm
        if (gamepad2.dpad_up) {
            AL.setPosition(1);
            AR.setPosition(1);
            telemetry.addData("Keypad", "Arm is up");

        } else if (gamepad2.dpad_down) {
            AL.setPosition(0.3);
            AR.setPosition(0.21);
            telemetry.addData("Keypad", "Arm is Down");

        } //else if (gamepad2.dpad_left) {
            //AL.setPosition(0.95);
            //AR.setPosition(0.8);
            //telemetry.addData("Keypad", "Arm is Low");
       // }
        // else if (gamepad2.dpad_right) {
            //AL.setPosition(0.3);
            //AR.setPosition(0.21);
            //telemetry.addData("Keypad", "Arm is Mid");
        //}

        //Linear
        double armstick = gamepad2.left_stick_y;
        if (gamepad2.left_trigger >= 0.25) {

            for (DcMotor slide : SD) {
                slide.setPower(0.4); // Down
            }


        } else if (gamepad2.right_trigger >= 0.4) {

            for (DcMotor slide : SD) {
                slide.setPower(-0.4);// Up
            }

        } else {

            for (DcMotor slide : SD) {
                slide.setPower(-0.04);// no
            }

        }


    }
}



