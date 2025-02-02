package org.firstinspires.ftc.teamcode;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name = "TeleOP") // On the Driver Hub, when TeleOP is clicked, clicking "TeleOP" runs this program
public class Reapr_ITD_TeleOp_Copy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Lynx Module, to call both Control Hub and Expansion Hub, reducing the reaction time between yanking the joystick and the robot moving
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Declare our motors
        
        // Meccanum Drivetrain
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse left motors 
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        // motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        // motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        // motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        
        // Controls operated by Driver #2
        Servo claw = hardwareMap.get(Servo.class, "claw"); 
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm"); 
        DcMotor fourBarCH = hardwareMap.get(DcMotor.class, "fourBarCH");
        DcMotor fourBarEH = hardwareMap.get(DcMotor.class, "fourBarEH");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        
        fourBarEH.setDirection(DcMotorSimple.Direction.REVERSE);
        
        fourBarCH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fourBarEH.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        
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
            motorFrontLeft.setPower((y + x + rx) / denominator / dividePower);
            motorBackLeft.setPower((y - x + rx) / denominator / dividePower);
            motorFrontRight.setPower((y - x - rx) / denominator / dividePower);
            motorBackRight.setPower((y + x - rx) / denominator / dividePower);

            // fourBar control stuff            
            fourBarCH.setPower(gamepad2.left_stick_y);
            fourBarEH.setPower(gamepad2.left_stick_y);
            
            // move claw open on y if not already at lowest position.
            if (gamepad2.left_trigger > 0.1 || gamepad1.left_trigger > 0.1){
                claw.setPosition(0);
            }

            // move claw closed on a if not already at the highest position.
            if (gamepad2.right_trigger > 0.1 || gamepad1.right_trigger > 0.1){
                claw.setPosition(0.8);
            }
        
            if (gamepad2.b) {
                arm.setPower(1);
            }
            
            arm.setPower(-gamepad2.right_stick_y);
            
            if (gamepad2.x) {
                arm.setPower(-1);
            }
            
            if (gamepad2.dpad_right) {
                wrist.setPosition(0.25);
            }
        }
    }
}
