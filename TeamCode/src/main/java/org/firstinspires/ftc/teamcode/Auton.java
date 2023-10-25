package org.firstinspires.ftc.teamcode;  // Define the package for your code

import com.acmerobotics.roadrunner.geometry.Pose2d;  // Import necessary Road Runner and FTC classes
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.drive.;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Auton")
public class Auton extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, rearLeft, rearRight;  // Declare motor variables
    private SampleMecanumDrive drive;  // Declare a Road Runner drive variable
    private ElapsedTime runtime = new ElapsedTime();  // Create an ElapsedTime object to track time

    @Override
    public void runOpMode() {  // Main OpMode method
        frontLeft = hardwareMap.get(DcMotorEx.class, "Left_Front_Motor");  // Initialize motors
        frontRight = hardwareMap.get(DcMotorEx.class, "Right_Front_Motor");
        rearLeft = hardwareMap.get(DcMotorEx.class, "Left_Back_Motor");
        rearRight = hardwareMap.get(DcMotorEx.class, "Right_Back_Motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);  // Set motor directions
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        drive = new SampleMecanumDrive(hardwareMap);  // Create a Road Runner drive object using hardwareMap

        waitForStart();  // Wait for the start button to be pressed

        if (isStopRequested()) return;  // Exit if stop is requested

        // Define drive constraints (adjust for your robot)
        DriveConstraints constraints = new DriveConstraints(
                45.0, 30.0, 0.0,  // Maximum velocity, maximum acceleration, and maximum angular velocity
                Math.toRadians(180), Math.toRadians(180), 0.0  // Maximum heading (angle) in radians
        );

        drive.setDriveConstraints(constraints);  // Set drive constraints for the Road Runner drive

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(1.0)  // Create a trajectory to move 1 meter forward
                .strafeRight(2.0)  // Add a segment to move 2 meters to the right
                .build();  // Build the trajectory

        drive.followTrajectory(trajectory);  // Execute the trajectory

        while (opModeIsActive() && !isStopRequested()) {
            // Your TeleOp control code goes here
            // You can add driver controls or other functionalities
        }
    }
}
