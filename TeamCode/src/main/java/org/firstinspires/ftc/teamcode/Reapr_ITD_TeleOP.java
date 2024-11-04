/* Base code from:
https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

Main TeleOP (2 Driver)

Ports:
Motors:
- Spinner Motor - Control 0
- Front Left Wheel - Control 1
- Back Left Wheel - Control 2
- Muscle - Control 3
- Worm Gear - Expansion 0
- Front Right Wheel - Expansion 1
- Back Right Wheel - Expansion 2
- Spool - Expansion 3 (not using anymore)

Servos: 
- none - Control 0
- Intake - Control 1
- Ramp - Control 2
- Drone Launcher - Control 3
- Hinge - Control 4
- Auton Servo - Control 5
- Arm - Expansion 0
- none - Expansion 1
- none - Expansion 2
- Bucket - Expansion 3
- none - Expansion 4
- none - Expansion 5
- 
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOP")

public class Reapr_ITD_TeleOP extends LinearOpMode {
    //    double hingePosition;
    double MIN_POSITION = 0, MAX_POSITION = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);  // This was connected on the expansion hub, it needs to be reversed
//
//        DcMotor wormGear = hardwareMap.dcMotor.get("wormGear"); // Port 0
//        // DcMotor spinner = hardwareMap.dcMotor.get("spinnerMotor"); // Port 0 (intake is a servo now)
//        CRServo intake = hardwareMap.crservo.get("intake");
//        CRServo ramp = hardwareMap.crservo.get("ramp");
//        CRServo autonServo = hardwareMap.crservo.get("autonDropServo");
//
       //Servo claw = hardwareMap.servo.get("claw");
       //DcMotor muscle = hardwareMap.dcMotor.get("muscle");
       DcMotor slide = hardwareMap.dcMotor.get("slide");
       //CRServo arm = hardwareMap.crservo.get("arm");
//
//        //DcMotor spool = hardwareMap.dcMotor.get("spool"); // Port 3
//
//        // Airplane launcher setup
//        CRServo launcher = hardwareMap.crservo.get("launcher");
//        CRServo bucketArm = hardwareMap.crservo.get("bucketArm");
//        CRServo bucket = hardwareMap.crservo.get("bucket");
//
//
//        /*
//        double launcherPosition = 0.5;
//        final double launcherSpeed = 0.1;// change to 100th when button is hold
//        final double launcherMinRange = 0.3;
//        final double launcherMaxRange = 0.55;
//        */
//
//        // Rack and pinion setup
//
//
        waitForStart();
//
//        hingePosition = 0.1;
//
       if (isStopRequested()) return;
//
        boolean isSlowMode = false;
        double dividePower = 1.0;
//
//        int currentPosition = 0;
//
        boolean keepMoving = false;
//
        while (opModeIsActive()) {
            //! Control Speed
            if (isSlowMode) {
                dividePower = 1.5;
            } else {
                dividePower = 1.0;
            }

            if (gamepad1.left_stick_button || gamepad1.dpad_right) {
                if (isSlowMode) {
                    isSlowMode = false;
                    sleep(500);
                } else {
                    isSlowMode = true;
                    sleep(500);
                }
            }

            //! Mecccanum controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator / dividePower; //Positive rotation results in forward & right motion
            double backLeftPower = (y - x + rx) / denominator / dividePower; //Positive rotation results in forward & left motion
            double frontRightPower = (y - x - rx) / denominator / dividePower; //Positive rotation results in forward & left motion
            double backRightPower = (y + x - rx) / denominator / dividePower; //Positive rotation results in forward & right motion

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            
            // arm control stuff
            double g2ly = gamepad2.left_stick_y;
            double g2lx =  gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double g2rx = gamepad2.right_stick_x;
            
            slide.setPower(g2ly / dividePower);         
        }
    }
}