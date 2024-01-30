package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Autonomous.drive.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    NormalizedColorSensor Color_detect;

    // other constants
    double p = 0.6;
    double top = 0.764;
    double mid = 0.117;
    double bottom= 0.086;

    int slide_pos = 0;
    int current_p = 0;
    final float[] hsv = new float[3] ;
    String AllianceColor;
    String arm_pos;

    ArrayList<String> pixlecolors = new ArrayList<String>();
    int pixlecount = pixlecolors.toArray().length;

    boolean centric = true;
    boolean tank = false;



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
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //bucket arm
        dropper = hardwareMap.servo.get("dropper");
        dropper.resetDeviceConfigurationForOpMode();
        dropper.setPosition(mid);
        arm_pos = "mid";


        //launcher
        drone = hardwareMap.servo.get("drone");
        drone.resetDeviceConfigurationForOpMode();

        //LIGHTS

        Lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");
        Lights.resetDeviceConfigurationForOpMode();
        pixlecolors.add("None");
        pixlecolors.add("None");
        AllianceColor = "None";
        Color_detect = hardwareMap.get(NormalizedColorSensor.class,"colorsensor" );
        Color_detect.resetDeviceConfigurationForOpMode();




        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_left){
                AllianceColor = "red";
            }
            else if(gamepad1.dpad_right){
                AllianceColor = "blue";
            }

            if(gamepad1.start && !centric){
                centric = true;
                //tank = false;
            }

           // if(gamepad1.back && !tank){
               // centric = false;
                //tank = true;
           // }

            if (centric){
                FieldCentric();
           }
           // else if(tank){
                //Mechnum_Drive();
           // }

            detect();
            display(AllianceColor);
            //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            telemetry.update();

        }
    }
    public void Mechnum_Drive(){
        //Reverse

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
    public void FieldCentric(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
        Operations();


    }
    public void Operations() {
        //Driver 1
        //roller
        if (gamepad1.left_trigger>=0.4) {
            roller.setPower(0.99);

            }
        else if (gamepad1.right_trigger>=0.4) {
            roller.setPower(-0.99);

        }
        else{
            roller.setPower(0.0);
            }

        //Driver 2
        //Drone Launch
        if(gamepad2.back){
            drone.setPosition(0);
        }

        //Arm
        //low
        if(gamepad2.a){
            dropper.setPosition(bottom);
            arm_pos = "bottom";
        }
        //mid
        if(gamepad2.x){
            returnmid();
            arm_pos = "mid";
        }
        //high
        if(gamepad2.y){
            deposit();
            del(pixlecolors,0,1);
            arm_pos = "TOP!!";
        }

        //Linear slide
        if(gamepad2.right_trigger > 0.4){
            slide.setPower(-0.75);
        } else if(gamepad2.left_trigger > 0.4){
            slide.setPower(0.5);
        } else{
            slide.setPower(-0.05);
        }
        telemetry.addData("Arm Position",  arm_pos);
    }

    public void up (int pos){

        slide_pos = pos + 1;

    }

    public void down(int pos){
        slide_pos = pos - 1;

    }

    public void returnmid(){
        //slide_pos = 0;
        dropper.setPosition(mid);
        //slide.setTargetPosition(slide_pos);
        //slide.setPower(0.55);

    }
    public void deposit(){
        //slide_pos = -250;
        // slide.setTargetPosition(slide_pos);
        //slide.setPower(0.55);
        //if (slide.getCurrentPosition()==250){
        dropper.setPosition(top);
        //}

    }
    public boolean check(double num1 , double num2, double val){
        if(val <= num1 && val >= num2 ){
            return true;
        }
        return false;

    }

    public void del(ArrayList<String> List,int pix1,int pix2){;
        List.set(pix1,"None");
        List.set(pix2,"None");

    }

    public void detect() {
        //Distance At 1 1/4 in

        NormalizedRGBA colors = Color_detect.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);


        if (check(190, 170, hsv[0]) &&
                check(0.6, 0.3, hsv[1]) &&
                check(0.08, 0.04, hsv[2])) {
            //last val 171.429 , 0.438 , 0.063
            pixlecolors.set(0,"white");
        } else if (check(230, 200, hsv[0]) &&
                check(1, 0.6, hsv[1]) &&
                check(0.05, 0.01, hsv[2])) {
            //last val 227 , 0.8 , 0.039
            pixlecolors.set(0,"purple");
        } else if (check(100, 70, hsv[0]) &&
                check(0.8, 0.5, hsv[1]) &&
                check(0.03, 0.01, hsv[2])) {
            pixlecolors.set(0,"yellow");
            //last val 171.429 , 0.714 , 0.0027
        } else if (check(150, 120, hsv[0]) &&
                check(0.9, 0.7, hsv[1]) &&
                check(0.03, 0.01, hsv[2])) {
            //last val 135 , 0.8 , 0.02
            pixlecolors.set(0,"green");
        }
    }
     public void display(String AllianceColor){

     
         telemetry.addLine()
                 .addData("Hue", "%.3f", hsv[0])
                 .addData("Saturation", "%.3f", hsv[1])
                 .addData("Value", "%.3f", hsv[2]);

        telemetry.addData("color sensor",  AllianceColor);
        telemetry.addData("OldPixle",  pixlecolors.get(0));
        telemetry.addData("New",  pixlecolors.get(1));


        if (pixlecolors.get(0).equalsIgnoreCase("white")){
            Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
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
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            }
            else {
                Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            }


        }
        }
    }





