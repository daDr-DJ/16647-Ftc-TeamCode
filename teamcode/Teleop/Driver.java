package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.apache.commons.math3.optim.PointVectorValuePair;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

@TeleOp

public class Driver extends LinearOpMode {
    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;

    DcMotor roller;

    DcMotor slide;

    Servo drone;

    Servo dropper;

    RevBlinkinLedDriver Lights;

    ColorSensor Color_detect;

    // other constants
    double p = 0.6;
    String AllianceColor;

    ArrayList<String> pixlecolors = new ArrayList<String>();
    int pixlecount = pixlecolors.toArray().length;

    @Override
    public void runOpMode() throws InterruptedException {
        //wheels
        LF = hardwareMap.dcMotor.get("left front");
        RF = hardwareMap.dcMotor.get("right front");
        LB = hardwareMap.dcMotor.get("left back");
        RB = hardwareMap.dcMotor.get("right back");

        //intake
        roller = hardwareMap.dcMotor.get("roller");



        //slides
        slide = hardwareMap.dcMotor.get("slide");

        //servo
        dropper = hardwareMap.servo.get("dropper");
        dropper.resetDeviceConfigurationForOpMode();
        drone = hardwareMap.servo.get("drone");
        drone.resetDeviceConfigurationForOpMode();


        //Motor reverse
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        Lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");
        Lights.resetDeviceConfigurationForOpMode();
        pixlecolors.add("None");
        pixlecolors.add("None");
        AllianceColor = "None";


        Color_detect = hardwareMap.colorSensor.get("colorsensor" );
        Color_detect.resetDeviceConfigurationForOpMode();




        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.back){
                AllianceColor = "red";
            }
            else if(gamepad2.start){
                AllianceColor = "blue";
            }

            Mechnum_Drive();
            display(AllianceColor);

            //Operations();

            telemetry.update();

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
        //Driver 1
        //roller
        if (gamepad1.left_trigger>=0.4) {
            roller.setPower(0.7);
            detect();
                telemetry.addData("gamepad1", "Rolling");
            }

        else{
            roller.setPower(0.0);
                telemetry.addData("gamepad1", "Stopped Rolling");
            }

       // if (pressed){


        //}
        //else if (!pressed) {

        // }

            //} else if (gamepad1.left_trigger > 0.4) {
            //roller.setPower(-0.75);
            //Colortimer();
            //telemetry.addData("gamepad1", "Rolling");



        //Driver 2
        //Drone Launch
        if(gamepad2.back){
            drone.setPosition(0);
        }

        //arm
        //low
        if(gamepad2.a){
            dropper.setPosition(0.99);

        }
        //mid
        if(gamepad2.x){
            dropper.setPosition(.95 );

        }
        //high
        if(gamepad2.y){
            dropper.setPosition(0);
            del(pixlecolors,0,2);
        }

        //Linear slide
        if(gamepad2.right_trigger > 0.4){
            slide.setPower(-0.6);
        }
        else if(gamepad2.left_trigger > 0.4){
            slide.setPower(0.6);
        }
        else{
            slide.setPower(-0.09);
        }


    }

    public boolean check(double num1 , double num2, double val){
        if(val <= num1 && val >= num2 ){
            return true;
        }
        return false;

    }

    public void del(ArrayList<String> List,int pix1,int pix2){
        List.remove(pix1);
        List.add(0,"None");

    }

    public void detect(){
        //Distanc e At 1 1/4 in

        if (check(-83000000, -100500000, Color_detect.argb())) {
            pixlecolors.add(0,"white");
        } else if (check(-300000000, -500000000, Color_detect.argb())) {
            pixlecolors.add(0,"purple");
        } else if (check(-100600000, -200000000, Color_detect.argb())) {
            pixlecolors.add(0,"green");
        } else if (check(-500005000, -704000000, Color_detect.argb())) {
            pixlecolors.add(0,"yellow");
        }
    }
    public void display(String AllianceColor){




        telemetry.addData("Red", Color_detect.red());
        telemetry.addData("Green", Color_detect.green());
        telemetry.addData("Blue", Color_detect.blue());
        telemetry.addData("rgb", Color_detect.argb());
        telemetry.addData("color sensor",  AllianceColor);
        telemetry.addData("OldPixle",  pixlecolors.get(0));
        telemetry.addData("New",  pixlecolors.get(1));


        if (pixlecolors.get(0).equalsIgnoreCase("white")){
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
            telemetry.addData("color sensor", "Pixel is White");
        }
        else if (pixlecolors.get(0).equalsIgnoreCase("purple")){
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            telemetry.addData("color sensor", "Pixel is purple");
        }

        else if (pixlecolors.get(0).equalsIgnoreCase("green")) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        else if (pixlecolors.get(0).equalsIgnoreCase("yellow")) {
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else if (pixlecolors.get(0).equalsIgnoreCase("None")){
            if (AllianceColor.equalsIgnoreCase("RED")) {
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            else {
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }


        }
        }
    }

        



