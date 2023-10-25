package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurePursuit extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private static final double WHEEL_DIAMETER = 4.0; // The diameter of your wheels in inches
    private static final double TICKS_PER_REVOLUTION = 1440; // Ticks per revolution for your motors
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Circumference of the wheels

    private static final double ROBOT_TRACK_WIDTH = 15.0; // The distance between the wheels (in inches)
    private static final double TRACK_WIDTH_OFFSET = 1.0; // Adjust as needed

    private static final double LOOKAHEAD_DISTANCE = 12.0; // Adjust as needed
    private static final double MAX_SPEED = 0.5; // Maximum motor speed

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors and other hardware here
        leftFront = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Reverse motors if needed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Your path or waypoints should be defined here
        double[][] waypoints = {{0, 0}, {24, 0}, {24, 48}, {0, 48}};

        // Loop through waypoints and navigate to them using Pure Pursuit
        for (int i = 0; i < waypoints.length; i++) {
            if (isStopRequested()) {
                return;
            }

            double targetX = waypoints[i][0];
            double targetY = waypoints[i][1];

            while (!isAtWaypoint(targetX, targetY)) {
                double[] targetPoint = calculateTargetPoint(targetX, targetY);
                double leftSpeed = calculateLeftWheelSpeed(targetPoint);
                double rightSpeed = calculateRightWheelSpeed(targetPoint);

                setMotorPowers(leftSpeed, rightSpeed);
            }

            stopMotors();
        }
    }

    private double[] calculateTargetPoint(double targetX, double targetY) {
        // Calculate the target point based on the current waypoint and lookahead distance
        // You can use geometric calculations to find the point
        // Return the target point as {targetX, targetY}
        return new double[]{0.2, 0.4};
    }

    private boolean isAtWaypoint(double targetX, double targetY) {
        // Check if the robot is within a certain tolerance of the current waypoint
        // Implement based on your odometry or sensors
        // Return true when the robot reaches the waypoint
        return true;
    }

    private double calculateLeftWheelSpeed(double[] targetPoint) {
        // Calculate the left wheel speed based on Pure Pursuit algorithm
        // Implement your control logic here
        return 0.0; // Adjust this value
    }

    private double calculateRightWheelSpeed(double[] targetPoint) {
        // Calculate the right wheel speed based on Pure Pursuit algorithm
        // Implement your control logic here
        return 0.0; // Adjust this value
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }

    private void stopMotors() {
        setMotorPowers(0.0, 0.0);
    }
}

