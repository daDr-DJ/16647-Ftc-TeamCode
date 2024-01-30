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
public class OpenCVAutoBlueClose extends LinearOpMode {
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


        Trajectory TrajBlueLeft1 = drive.trajectoryBuilder(startPose)
                .strafeRight(30,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();

        Trajectory TrajBlueLeft1_1= drive.trajectoryBuilder(TrajBlueLeft1.end())

                .back(4,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.2)
                )
                .addDisplacementMarker(0.4, () -> {
                    roller.setPower(-0.24);
                    sleep(700);
                    roller.setPower(0);

                })

                .build();


        Trajectory TrajBlueLeft2 = drive.trajectoryBuilder(TrajBlueLeft1_1.end().plus(new Pose2d(0, 0, Math.toRadians(-195))))
                .strafeRight(5,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.2)
                )

                .build();


        Trajectory TrajBlueLeft3 = drive.trajectoryBuilder(TrajBlueLeft2.end())
                .back(14,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();
        Trajectory TrajBlueLeft4= drive.trajectoryBuilder(TrajBlueLeft3.end())
                .strafeRight(13,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();

        Trajectory TrajBlueLeft5= drive.trajectoryBuilder(TrajBlueLeft4.end())
                .back(25,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();

        Trajectory TrajBlueLeft6= drive.trajectoryBuilder(TrajBlueLeft5.end())
                .strafeLeft(8,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.4)
                )

                .build();


        Trajectory TrajBlueLeft7 = drive.trajectoryBuilder(TrajBlueLeft6.end())
                .back(20
                        ,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL*0.5)
                )

                .addDisplacementMarker(19, () -> {

                    // Sleep for 1500ms
                    sleep(1200);
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);

                })
                .build();


        Trajectory TrajBlueMid1 = drive.trajectoryBuilder(startPose)

                .strafeRight(26)//26

                .build();
        Trajectory TrajBlueSpike = drive.trajectoryBuilder(TrajBlueMid1.end().plus(new Pose2d(0,0,Math.toRadians(-100))))

                .back(2,
                        SampleMecanumDrive.getVelocityConstraint(4, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.4)
                )
                .addDisplacementMarker(-0.2, () -> {
                    roller.setPower(-0.24);
                    sleep(600);
                    roller.setPower(0);

                })
                .build();

        Trajectory TrajBlueMid2 = drive.trajectoryBuilder(TrajBlueSpike.end().plus(new Pose2d(0, 0, Math.toRadians(-95))))
                .back(42,
                        SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.9)
                )


                .build();

        Trajectory TrajBlueMid3 = drive.trajectoryBuilder(TrajBlueMid2.end())
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )
                .addTemporalMarker(0.5, () -> {


                    // Sleep for 1500ms
                    sleep(1200);
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);


                })

                .build();

        Trajectory TrajBlueRight1 = drive.trajectoryBuilder(startPose)
                .strafeRight(30)

                .build();



        Trajectory TrajBlueRight3 = drive.trajectoryBuilder(TrajBlueRight1.end().plus(new Pose2d(0,0,Math.toRadians(-195))))
                .addTemporalMarker(-0.4, () -> {
                    roller.setPower(-0.55);
                    sleep(600);
                    roller.setPower(0);

                })
                .back(52,
                        SampleMecanumDrive.getVelocityConstraint(13, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5)
                )

                .addDisplacementMarker(51, () -> {
                    dropper.setPosition(top);
                    sleep(1000);
                    dropper.setPosition(mid);
                    sleep(1000);

                })


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
                drive.followTrajectory(TrajBlueLeft1);
                drive.followTrajectory(TrajBlueLeft1_1);
                drive.turn(Math.toRadians(-195));
                drive.followTrajectory(TrajBlueLeft2);
                drive.followTrajectory(TrajBlueLeft3);
                drive.followTrajectory(TrajBlueLeft4);
                drive.followTrajectory(TrajBlueLeft5);
                drive.followTrajectory(TrajBlueLeft6);
                drive.followTrajectory(TrajBlueLeft7);




            } else if (marker_pos == 1) {
                drive.followTrajectory(TrajBlueMid1);
                drive.turn(Math.toRadians(-100));
                drive.followTrajectory(TrajBlueSpike);
                drive.turn(Math.toRadians(-95));
                drive.followTrajectory(TrajBlueMid2);
                drive.followTrajectory(TrajBlueMid3);
            }
            //drive.trajectorySequenceBuilder(new Pose2d(-14, -60, Math.toRadians(180)))
            //.splineToConstantHeading(new Vector2d(-14,-45),Math.toRadians(180))
            //.splineTo(new Vector2d(-16,-33),Math.toRadians(90))
            //.back(-12)
            // .turn(Math.toRadians(90))
            //.splineToConstantHeading(new Vector2d(-48,-43),Math.toRadians(270))
            //.splineToConstantHeading(new Vector2d(-48,-35),Math.toRadians(270))
            //.build();
            //drive.followTrajectory(TrajRedMid1);
            //drive.turn(Math.toRadians(-90));
            //drive.followTrajectory(TrajRedMid2);
            //drive.followTrajectory(TrajRedMid3);
            //drive.turn(Math.toRadians(90));
            //drive.followTrajectory(TrajRedMid4);
            //drive.followTrajectory(TrajRedMid5);


            else if (marker_pos == 2) {
                drive.followTrajectory(TrajBlueRight1);
                drive.turn(Math.toRadians(-195));
                drive.followTrajectory(TrajBlueRight3);



            } else {
                drive.followTrajectory(TrajBlueMid1);
                drive.turn(Math.toRadians(-195));
                drive.followTrajectory(TrajBlueMid2);
                drive.followTrajectory(TrajBlueMid3);


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
        Scalar rectColor = new Scalar(0, 0.0, 255.0);


        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 100, 212, 179);
            Rect midRect = new Rect(213, 100, 212, 179);
            Rect rightRect = new Rect(426, 100, 212, 179);

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

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
                marker_pos = 0;

            } else if (midavgfin > leftavgfin && midavgfin > rightavgfin) {
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

