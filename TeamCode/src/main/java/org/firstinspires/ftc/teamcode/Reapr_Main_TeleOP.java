/* Base code from:
https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

Main TeleOP (1 Driver)
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOP")

public class Reapr_Main_TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Port 0
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Port 1
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Port 2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Port 3

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);  // This was connected on the expansion hub, it needs to be reversed


        DcMotor spinnerMotor = hardwareMap.dcMotor.get("spinnerMotor"); // Port 0


        waitForStart();

        if (isStopRequested()) return;

        boolean isSlowMode = false;
        double dividePower=1.0;

        while (opModeIsActive()) {
            // Control Speed
            if(isSlowMode){
                dividePower=1.5;
            }else{
                dividePower=1.0;
            }

            if(gamepad1.left_stick_button){
                if(isSlowMode){
                    isSlowMode=false;
                    sleep(500);
                }else{
                    isSlowMode=true;
                    sleep(500);
                }
            }

            // Mecccanum controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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



            // Spinner motor Controls


            if (gamepad1.a){ // Move down
                spinnerMotor.setPower(1);
            }
            spinnerMotor.setPower(0);

            if (gamepad1.y){ // Move up
                spinnerMotor.setPower(-1);
            }
            spinnerMotor.setPower(0);

        }
    }
}
