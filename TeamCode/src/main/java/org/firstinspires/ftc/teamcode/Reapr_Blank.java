package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Literally Does Nothing :)") // On the Driver Hub, when Autonomous is clicked, clicking "Literally Does Nothing :)" runs this program

// This is a program which runs a "Auton", which pauses the robot for the full Auton duration

public class Reapr_Blank extends LinearOpMode {
    public void runOpMode() {
        waitForStart(); // Everything underneath this will occur when play button is clicked
        sleep(30000); // Pauses the robot for a period of 30,000 seconds, equivilant to 30 seconds, or the full Auton duratiom
    }
}
