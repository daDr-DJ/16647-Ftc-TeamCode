package org.firstinspires.ftc.teamcode.Autonomous.drive.drive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class OpenCVAutoRedClose extends LinearOpMode {
    OpenCvWebcam webcam1 = null;
    DcMotor roller;

    DcMotor slide;

    Servo dropper;
    String arm_pos;
    double top = 0.8;
    double mid = 0.2;

    int marker_pos;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);//14, 60, 0);
        drive.setPoseEstimate(startPose);


        Trajectory TrajRedLeft1 = drive.trajectoryBuilder(startPose)
                .back(7,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )

                .build();

        Trajectory TrajRedLeft2 = drive.trajectoryBuilder(TrajRedLeft1.end())
                .strafeRight(28.6,
                        SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )

                .addDisplacementMarker(28.5, () -> {
                    roller.setPower(-0.75);
                    sleep(600);
                    roller.setPower(0);

                })

                .build();


        Trajectory TrajRedLeft3 = drive.trajectoryBuilder(TrajRedLeft2.end())

                .back(21,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )



                .build();


        Trajectory TrajRedLeft4 = drive.trajectoryBuilder(TrajRedLeft3.end())

                .strafeRight(5,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )



                .build();

        Trajectory TrajRedLeft5 = drive.trajectoryBuilder(TrajRedLeft4.end())
                .back(19,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)

                )
                .addDisplacementMarker(18, () -> {
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);

                })
                .build();

        Trajectory TrajRedLeft6 = drive.trajectoryBuilder(TrajRedLeft5.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                                .build();

        Trajectory TrajRedLeft7 = drive.trajectoryBuilder(TrajRedLeft6.end())
                .strafeLeft(30,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();

        Trajectory TrajRedLeft8 = drive.trajectoryBuilder(TrajRedLeft7.end())
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();



        Trajectory TrajRedMidBack = drive.trajectoryBuilder(startPose)
                .back(8.5,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9)
                )


                .build();

        Trajectory TrajRedMid1 = drive.trajectoryBuilder(TrajRedMidBack.end())

                .strafeRight(26)
                //.splineToConstantHeading(new Vector2d(14,45),Math.toRadians(0))
                .build();


        Trajectory TrajRedSpike = drive.trajectoryBuilder(TrajRedMid1.end().plus(new Pose2d(0,0,Math.toRadians(-95))))
                .addTemporalMarker(-0.4, () -> {
                    roller.setPower(-0.54);
                    sleep(600);
                    roller.setPower(0);

                })

                .forward(0.01,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.4)
                )

                .build();

        Trajectory TrajRedMid2 = drive.trajectoryBuilder(TrajRedSpike.end().plus(new Pose2d(0, 0, Math.toRadians(95))))
                .back(20,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9)
                )


                .build();

        Trajectory TrajRedMid3 = drive.trajectoryBuilder(TrajRedMid2.end())
                .strafeRight(1,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.4)
                )


                .build();

        Trajectory TrajRedMid4 = drive.trajectoryBuilder(TrajRedMid3.end())
                .back(14,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9)
                )


                .build();

        Trajectory TrajRedMid5 = drive.trajectoryBuilder(TrajRedMid4.end())
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )
                .addDisplacementMarker(4, () -> {

                    // Sleep for 1500ms
                    sleep(1200);
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);


                })

                .build();

       Trajectory TrajRedMid6 = drive.trajectoryBuilder(TrajRedMid5.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();

        Trajectory TrajRedMid7 = drive.trajectoryBuilder(TrajRedMid6.end())
                .strafeLeft(30,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();

        Trajectory TrajRedMid8 = drive.trajectoryBuilder(TrajRedMid7.end())
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();


        Trajectory TrajRedRight1 = drive.trajectoryBuilder(startPose)
                .back(23,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();

        Trajectory TrajRedRight1_1= drive.trajectoryBuilder(TrajRedRight1.end())
                .strafeRight(28,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5)
                )


                .build();


        Trajectory TrajRedRight2 = drive.trajectoryBuilder(TrajRedRight1_1.end())


                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.2)
                )
                .addDisplacementMarker(-.4, () -> {
                    roller.setPower(-0.25);
                    sleep(700);
                    roller.setPower(0);

                })

                .build();



        Trajectory TrajRedRight3 = drive.trajectoryBuilder(TrajRedRight2.end())
                .strafeLeft(7.5,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();




        Trajectory TrajRedRight4 = drive.trajectoryBuilder(TrajRedRight3.end())
                .back(12.5
                        ,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5)
                )

                .addDisplacementMarker(12, () -> {

                    // Sleep for 1500ms
                    sleep(1000);
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);


                })
                .build();

        Trajectory TrajRedRight5 = drive.trajectoryBuilder(TrajRedRight4.end())
                .forward(8,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();

        Trajectory TrajRedRight6 = drive.trajectoryBuilder(TrajRedMid5.end())
                .strafeLeft(30,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();

        Trajectory TrajRedRight7 = drive.trajectoryBuilder(TrajRedRight6.end())
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))

                .build();




        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Alinity");
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("CameraMotionField",
                "id", hardwareMap.appContext.getPackageName());

        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);
        webcam1.setPipeline(new examplePipeline());


        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }

        });

        //intake
        roller = hardwareMap.dcMotor.get("roller");

        //slides

        slide = hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //bucket arm
        dropper = hardwareMap.servo.get("dropper");
        dropper.resetDeviceConfigurationForOpMode();
        dropper.setPosition(mid);
        arm_pos = "mid";


        waitForStart();

        webcam1.stopStreaming();

        if (!isStopRequested()) {

            if (marker_pos == 0) {
                drive.followTrajectory(TrajRedLeft1);
                drive.followTrajectory(TrajRedLeft2);
                drive.followTrajectory(TrajRedLeft3);
                drive.followTrajectory(TrajRedLeft4);
                drive.followTrajectory(TrajRedLeft5);
                drive.followTrajectory(TrajRedLeft6);
                drive.followTrajectory(TrajRedLeft7);
                drive.followTrajectory(TrajRedLeft8);






            } else if (marker_pos == 1) {
                drive.followTrajectory(TrajRedMidBack);
                drive.followTrajectory(TrajRedMid1);
                drive.turn(Math.toRadians(-95));
                drive.followTrajectory(TrajRedSpike);
                drive.turn(Math.toRadians(95));
                drive.followTrajectory(TrajRedMid2);
                drive.followTrajectory(TrajRedMid3);
                drive.followTrajectory(TrajRedMid4);
                drive.followTrajectory(TrajRedMid5);
                drive.followTrajectory(TrajRedMid6);
                drive.followTrajectory(TrajRedMid7);
                drive.followTrajectory(TrajRedMid8);


            }

            else if (marker_pos == 2) {
                drive.followTrajectory(TrajRedRight1);
                drive.followTrajectory(TrajRedRight1_1);
                drive.followTrajectory(TrajRedRight2);
                drive.followTrajectory(TrajRedRight3);
                drive.followTrajectory(TrajRedRight4);
                drive.followTrajectory(TrajRedRight5);
                drive.followTrajectory(TrajRedRight6);
                drive.followTrajectory(TrajRedRight7);



            } else {
                drive.followTrajectory(TrajRedMidBack);
                drive.followTrajectory(TrajRedMid1);
                drive.turn(Math.toRadians(-95));
                drive.followTrajectory(TrajRedSpike);
                drive.turn(Math.toRadians(95));
                drive.followTrajectory(TrajRedMid2);
                drive.followTrajectory(TrajRedMid3);
                drive.followTrajectory(TrajRedMid4);
                drive.followTrajectory(TrajRedMid5);
                drive.followTrajectory(TrajRedMid6);
                drive.followTrajectory(TrajRedMid7);
                drive.followTrajectory(TrajRedMid8);




            }


        }
    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 125, 212, 179);
            Rect midRect = new Rect(213, 125, 212, 179);
            Rect rightRect = new Rect(427, 125, 212, 179);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, midRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);


            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin < midavgfin && leftavgfin < rightavgfin) {
                telemetry.addLine("Left");
                marker_pos = 0;

            } else if (midavgfin < leftavgfin && midavgfin < rightavgfin) {
                telemetry.addLine("middle");
                marker_pos = 1;

            } else {
                telemetry.addLine("Right");
                marker_pos = 2;
            }
            telemetry.update();
            return (output);

        }
    }
}

