// Base code from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

package org.firstinspires.ftc.teamcode;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TeleOP") // On the Driver Hub, when TeleOP is clicked, clicking "TeleOP" runs this program
public class Reapr_ITD_TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Lynx Module, to call both Control Hub and Expansion Hub, reducing the reaction time between yanking the joystick and the robot moving
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // Sets the Caching Mode to AUTO, rather than OFF (default), which speeds up loop times.
        }

        // Declare our motors
        
        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // An alternative to  DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft"); // An alternative to  DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight"); // An alternative to  DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight"); // An alternative to  DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse left motors 
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Controls operated by Driver #2
        Servo claw = hardwareMap.get(Servo.class, "claw"); 
        CRServo arm = hardwareMap.get(CRServo.class, "arm"); 
        DcMotor fourBarCH = hardwareMap.get(DcMotor.class, "fourBarCH");
        DcMotor fourBarEH = hardwareMap.get(DcMotor.class, "fourBarEH");
        CRServo wrist = hardwareMap.get(CRServo.class, "wrist");
        
        waitForStart();

       if (isStopRequested()) return;
       
       fourBarCH.setPower(0);
       fourBarEH.setPower(0);
       
        double dividePower = 1.0;

        while (opModeIsActive()) {
            //! Control Speed
            if (gamepad1.left_stick_button || gamepad1.dpad_right) {
                dividePower = Math.abs(dividePower - 1.0) < 0.1 ? 1.5 : 1.0;
                sleep(500);
            }


            // Mecccanum controls
            double y = -gamepad1.left_stick_y;       // Remember, this is reversed!
            double x =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motorFrontLeft.setPower((y + x + rx) / denominator / dividePower);  //Positive rotation results in forward & right motion
            motorBackLeft.setPower((y - x + rx) / denominator / dividePower);   //Positive rotation results in forward & left motion
            motorFrontRight.setPower((y - x - rx) / denominator / dividePower); //Positive rotation results in forward & left motion
            motorBackRight.setPower((y + x - rx) / denominator / dividePower);  //Positive rotation results in forward & right motion

            // fourBar control stuff            
            fourBarCH.setPower(gamepad2.left_stick_y);
            fourBarEH.setPower(gamepad2.left_stick_y);
            
            // wrist control stuff            
            wrist.setPower(-gamepad2.right_stick_x / 1.5);
            
            // move claw open on y if not already at lowest position.
            if (gamepad2.y){
                claw.setPosition(0);
            }

            // move claw closed on a if not already at the highest position.
            if (gamepad2.a){
                claw.setPosition(1);
            }
        
            if (gamepad2.x) {
                arm.setPower(1);
            } else if (gamepad2.b) {
                arm.setPower(-1);
            }
            arm.setPower(0);
        }
    }
}