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
    double clawPosition;
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
        
        // Claw Motors (Servo)
        Servo claw = hardwareMap.servo.get("claw");// name of servo on control hub is claw
        CRServo arm = hardwareMap.crservo.get("arm");// name of servo on control hub is arm
        // On port:
        
        //final double clawSpeed = 0.05;// change to 100th when button is hold
        //final double clawMinRange = 0.0;
        //final double clawMaxRange = 0.55;
        
       DcMotor slide = hardwareMap.dcMotor.get("slide");
       
        waitForStart();
        
        clawPosition = 0.1;

       if (isStopRequested()) return;

        boolean isSlowMode = false;
        double dividePower = 1.0;

        boolean keepMoving = false;

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
            
            
            // claw

            // move claw open on y if not already at lowest position.
            if (gamepad2.y){   
                clawPosition = 0;
                claw.setPosition(clawPosition);
            }

            // move claw closed on a if not already at the highest position.
            if (gamepad2.a){
                //clawPosition += .01;
                //telemetry.addData("a pressed", "%.2f", clawPosition, "%.2f", MIN_POSITION, "%.2f", MAX_POSITION);
               // telemetry.update();
                clawPosition = 1;
                // claw.setPosition(Range.clip(clawPosition, MIN_POSITION, MAX_POSITION));
                claw.setPosition(clawPosition);
            } 

            telemetry.addData("claw postion ", "%.2f", clawPosition); //displays the values on the driver hub
            telemetry.update();
        
            if (gamepad2.x) {
                arm.setPower(1);
            } else if (gamepad2.b) {
                arm.setPower(-1);
            } else {
                arm.setPower(0); // Ensure arm stops when buttons are not pressed
            } 
          
        }
    }
}