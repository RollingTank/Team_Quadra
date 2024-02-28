package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurplePixelDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Camera;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueClose;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueFar;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRedClose;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRedFar;

import java.util.List;

@Config
@Autonomous(name = "red far test")
public class RedFarCycle extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public static int loc = 1;

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {


        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        robot.init(hardwareMap);

       // robot.startCamera();
        Camera c = new Camera("RF");
        c.startCamera();



        Location location = c.returnLocation();

  /*      if (loc == 2) {
            location = Location.CENTER;
        } else if (loc == 3) {
            location = Location.RIGHT;
        }
*/
        Pose purplePose = new Pose();
        Pose yellowPose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        CommandScheduler.getInstance().schedule(new ArmCommand(160));


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        switch (location) {
            case LEFT:
                purplePose = new Pose(-3.5, 8.5, Math.toRadians(12.5));
                yellowPose = new Pose();
                break;
            case CENTER:
                purplePose = new Pose(-1.5, 7.6, Math.toRadians(2.5));
                yellowPose = new Pose();
                break;
            case RIGHT:
                purplePose = new Pose(-1, 1, Math.toRadians(-32));
                yellowPose = new Pose();
                break;
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelDropCommand()),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(200),

                        new PurpleDropRetractCommand(),

                        new PositionCommand(new Pose(-5, 25.5, Math.toRadians(90))),
                        new WhiteFrontExtendCommand(),
                        new WhiteFrontRetractCommand(),
                        new PathCommand(new Pose(-4, 3, Math.toRadians(90)), new Pose(45, 1, Math.toRadians(90)))
                )
        );


        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            robot.update();
            CommandScheduler.getInstance().run();
        }

        robot.kill();

    }
}
