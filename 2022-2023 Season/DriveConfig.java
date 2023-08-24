package teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.Arrays;
import java.util.List;


@Config

public class DriveConfig extends LinearOpMode{

    @NonNull

    DcMotor SL,SR;
    List<DcMotor> SD;

    @NonNull
    Servo CL;

    @NonNull
    CRServo AL;
    CRServo AR;

    @NonNull
    DcMotor LF;

    @NonNull
    DcMotor RF;

    @NonNull
    DcMotor LB;

    @NonNull
    DcMotor RB;




    @Override
    @NonNull
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
        AL = hardwareMap.crservo.get("arm left");
        AR = hardwareMap.crservo.get("arm right");
        //AR.resetDeviceConfigurationForOpMode();
        //AL.resetDeviceConfigurationForOpMode();

        //claw
        CL = hardwareMap.servo.get("claw left");
        CL.resetDeviceConfigurationForOpMode();


        //Motor reverse
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo reverse

        AR.setDirection(CRServo.Direction.REVERSE);
        //AL.setDirection(CRServo.Direction.FORWARD);




        SD = Arrays.asList(SL, SR);

        for (DcMotor slide : SD) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}