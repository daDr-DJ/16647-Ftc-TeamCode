package teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import java.util.Arrays;
import java.util.List;

@TeleOp
//@Disabled

public class driver extends LinearOpMode {

    double contpwrA;
    double contpwrC = 0;
    int currentpos;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConfig drive = new DriveConfig();

        //slide pos
        int reset = 0;
        int ground = 50;
        int low = 150;
        int S_pos = 0;

        // other constants
        double p = 0.6;

        waitForStart();

        while (opModeIsActive()) {

            //input values
            double ly = gamepad1.left_stick_y;
            double ry = gamepad1.right_stick_y;
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double lefttrig = gamepad2.left_trigger;
            double righttrig = gamepad2.right_trigger;
            double armstick = gamepad2.right_stick_y;


            //---------------------------------------------------

            //Forward & Back
            if (gamepad1.left_bumper) {
                drive.LF.setPower(((ly) - (lx)));
                drive.LB.setPower(((ly) + (lx)));
                drive.RF.setPower((ry) + (rx));
                drive.RB.setPower((ry) - (rx));

                //Straif Left & Right
            } else {
                drive.LF.setPower(((ly) * p - (lx) * p));
                drive.LB.setPower(((ly) * p + (lx) * p));
                drive.RF.setPower((ry) * p + (rx) * p);
                drive.RB.setPower((ry) * p - (rx) * p);
            }


            //---------------------------------------------------------

            //Slide controls
            if (gamepad2.b) {
                S_pos = reset;
            }

            else if (gamepad2.a) {
                S_pos = ground;
            }

            else if (gamepad2.x) {
                S_pos = low;
            }

            //if (gamepad2.y) {S_pos = mid;}


            if (righttrig >= 0.4){
                S_pos +=  1;
               //for (DcMotor slide : SD){
                    //slide.setPower(0.2);
               // }
            }

            else if (lefttrig >= 0.4){
                S_pos -=  1;
                //for (DcMotor slide : SD){
                    //slide.setPower(-0.2);
               //}
            }


            //-----------------------------------------------------------

            //Arm

            if (gamepad2.dpad_up){
                contpwrA = 0.35;
            }

            else if(gamepad2.dpad_down){
                contpwrA = 0.65;
            }

            else {
                contpwrA = 0.0;
            }

            //Claw
            if (gamepad2.left_bumper) {
                contpwrC = 0.53;
            }

            else if(gamepad2.right_bumper) {
                contpwrC = 0.472;
            }

            //---------------------------------------------------------

            drive.AR.setPower(contpwrA);
            drive.AL.setPower(contpwrA);
            drive.CL.setPosition(contpwrC);

            //for (DcMotor slide: drive.SD) {
                //slide.setTargetPosition(S_pos); // sets arm Height (90) degrees
                //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Assign motor to run to position\
                //slide.setPower(0.2);
             //}
        }
    }

}
