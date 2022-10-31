package ALL;
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

        A.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            
            //driving values
            double ly = gamepad1.left_stick_y;
            double ry = gamepad1.right_stick_y;
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double p = 0.5;


            //---------------------------------------------------
            
            //Forward & Back
            if (gamepad1.left_bumper) {
                LF.setPower((ly)-(lx));
                LB.setPower((ly)+(lx));
                RF.setPower((ry)+(rx));
                RB.setPower((ry)-(rx));
                
            //Straif Left & Right    
            } else {
                LF.setPower((ly)*p-(lx)*p);
                LB.setPower((ly)*p+(lx)*p);
                RF.setPower((ry)*p+(rx)*p);
                RB.setPower((ry)*p-(rx)*p);
            }


            //---------------------------------------------------------
            
            //Arm
            if (gamepad2.b) {
                pos_start();
            }

            if (gamepad2.a) {
                pos_term();
            }

            if (gamepad2.x) {
                pos_ground();
            }

            if (gamepad2.y) {
                pos_low();
            }

            if (gamepad2.left_trigger>0.1) {
                A.setPower(-0.3);
            } else {
                A.setPower(0);
            }
            //-----------------------------------------------------------

            //Grabber
            if (gamepad2.right_bumper) {
                GA.setPosition(-1.5);
                GB.setPosition(-1.5);
            }
            if (gamepad2.left_bumper) {
                GA.setPosition(1.5);
                GB.setPosition(1.5);
            }
            //---------------------------------------------------------

        }
    }
    String running = "motor is turning";
    
    // Arm Encoder Functions
    void pos_start() {

        A.setTargetPosition(0); // sets arm to 90 degrees

        A.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Assign motor to run to position

        A.setPower(0.4); // At 40% power


        // while (A.isBusy()) { System.out.print(running);}

    }

    void pos_term() {

        A.setTargetPosition(2056); // sets arm to 195 degrees

        A.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        A.setPower(0.4); 
        
        //while (A.isBusy()) { System.out.print(running);}
    }

    void pos_ground() {

        A.setTargetPosition(2110); // sets arm to 190 degrees

        A.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        A.setPower(0.4); 

        //while (A.isBusy()) { System.out.print(running);}

    }

    void pos_low() {

        A.setTargetPosition(379); // sets arm to 115 degrees

        A.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        A.setPower(0.4); 

        //while (A.isBusy()) { System.out.print(running);}

    }
    //------------------------------------------------------------

}
