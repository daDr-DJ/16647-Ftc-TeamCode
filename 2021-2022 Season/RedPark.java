package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
//@Disabled

public class RedPark extends LinearOpMode {

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
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
//change these things
        closeServo();
        lineUp();

    }
    void closeServo () {
        G.setPosition(0);
    }
    void openServo () {
        G.setPosition(1);
    }
    void lineUp () {
        move(900, 0.5);
        turn(1350, -0.5, 0.5);
        move(850, 0.5);
    }

    void move(int milliseconds, double power) {
        LF.setPower(power);
        RF.setPower(power);
        LB.setPower(power);
        RB.setPower(power);
        sleep(milliseconds);
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }
    void turn(int milliseconds, double powerLeft, double powerRight) {
        LF.setPower(powerLeft);
        RF.setPower(powerRight);
        LB.setPower(powerLeft);
        RB.setPower(powerRight);
        sleep(milliseconds);
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }
    void oneDuck () {
        D.setPower(0.2);
        sleep(750);
        D.setPower(0);
    }
}
