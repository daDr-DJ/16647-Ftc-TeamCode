package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="color sensor test", group="")
//@Disabled
public class hngh extends LinearOpMode{

    DcMotor lb;
    DcMotor lf;
    DcMotor rb;
    DcMotor rf;

    ColorSensor cs;

    @Override
    public void runOpMode(){
        lb = hardwareMap.get(DcMotor.class, "left back");
        lf = hardwareMap.get(DcMotor.class, "left front");
        rb = hardwareMap.get(DcMotor.class, "right back");
        rf = hardwareMap.get(DcMotor.class, "right front");

        cs = hardwareMap.get(ColorSensor.class, "color sensor");

        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            //activates led for color sensor
            cs.enableLed(true);


            //detects if an object is mostly red
            //IMPORTANT: rgb values have been proven unreliable, I will work on using alpha values (greyscale) which is better
            while ((cs.red() > 150) && (cs.blue() < 50) && (cs.green() < 50)){
                lf.setPower(.5);
                lb.setPower(.5);
                rf.setPower(.5);
                rb.setPower(.5);
            }
        }
    }
}
