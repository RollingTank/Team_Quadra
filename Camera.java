package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueClose;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueFar;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRedClose;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRedFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Camera {
    public Camera(String camSide){
        pipelineSetter = camSide;
    }
    RobotHardware robot;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private OpModeVisionRedFar pipeline;
    private OpModeVisionRedClose pipelin;
    private OpModeVisionBlueClose pipeli;
    private OpModeVisionBlueFar pipel;
    private List<LynxModule> allHubs;
    String pipelineSetter;
    OpenCvWebcam camera;
    org.firstinspires.ftc.teamcode.common.vision.Location l = org.firstinspires.ftc.teamcode.common.vision.Location.UNFOUND;
    public void startCamera() {
        //robot = RobotHardware.getInstance();
        //allHubs = hardwareMap.getAll(LynxModule.class);
     //   telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
      //  robot.init(hardwareMap);
      //  for (LynxModule hub : allHubs) {
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
     //   }
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //set camera settings here
                .build();

      //c  int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"));

        if(pipelineSetter.equals("BF")){
            pipel = new OpModeVisionBlueFar();
            camera.setPipeline(pipel);
            Globals.SIDE = org.firstinspires.ftc.teamcode.common.centerstage.Location.FAR;
            Globals.ALLIANCE = org.firstinspires.ftc.teamcode.common.centerstage.Location.BLUE;
            int p = convertLocation(pipel.location);
            switch (p){
                case 1:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.RIGHT;
                    telemetry.addLine("Right");
                    telemetry.update();
                    break;
                case 2:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.CENTER;
                    telemetry.addLine("Center");
                    telemetry.update();
                    break;
                case 3:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.LEFT;
                    telemetry.addLine("Left");
                    telemetry.update();
                    break;
            }

        }
        else if(pipelineSetter.equals("BC")){
            pipeli = new OpModeVisionBlueClose();
            camera.setPipeline(pipeli);
            Globals.SIDE = org.firstinspires.ftc.teamcode.common.centerstage.Location.CLOSE;
            Globals.ALLIANCE = org.firstinspires.ftc.teamcode.common.centerstage.Location.BLUE;
            int p = convertLocation(pipeli.location);
            switch (p){
                case 1:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.RIGHT;
                    telemetry.addLine("Right");
                    telemetry.update();
                    break;
                case 2:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.CENTER;
                    telemetry.addLine("Center");
                    telemetry.update();
                    break;
                case 3:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.LEFT;
                    telemetry.addLine("Left");
                    telemetry.update();
                    break;
            }
        }
        else if(pipelineSetter.equals("RF")){
            pipeline = new OpModeVisionRedFar();
            camera.setPipeline(pipeline);
            Globals.SIDE = org.firstinspires.ftc.teamcode.common.centerstage.Location.FAR;
            Globals.ALLIANCE = org.firstinspires.ftc.teamcode.common.centerstage.Location.RED;
            int p = convertLocation(pipeline.location);
            switch (p){
                case 1:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.RIGHT;
                    telemetry.addLine("Right");
                    telemetry.update();
                    break;
                case 2:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.CENTER;
                    telemetry.addLine("Center");
                    telemetry.update();
                    break;
                case 3:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.LEFT;
                    telemetry.addLine("Left");
                    telemetry.update();
                    break;
            }
        }
        else if(pipelineSetter.equals("RC")){
            pipelin = new OpModeVisionRedClose();
            camera.setPipeline(pipelin);
            Globals.SIDE = org.firstinspires.ftc.teamcode.common.centerstage.Location.CLOSE;
            Globals.ALLIANCE = org.firstinspires.ftc.teamcode.common.centerstage.Location.RED;
            int p = convertLocation(pipelin.location);
            switch (p){
                case 1:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.RIGHT;
                    telemetry.addLine("Right");
                    telemetry.update();
                    break;
                case 2:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.CENTER;
                    telemetry.addLine("Center");
                    telemetry.update();
                    break;
                case 3:
                    l = org.firstinspires.ftc.teamcode.common.vision.Location.LEFT;
                    telemetry.addLine("Left");
                    telemetry.update();
                    break;
            }
        }


        camera.setMillisecondsPermissionTimeout(Integer.MAX_VALUE); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("No Camera Could Not Start");
            }
        });
        //telemetry.addLine("Waiting for start");
        // telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Globals.IS_AUTO = true;

    }




























    public int convertLocation(OpModeVisionRedFar.TeamPropLocation a){
        if(a == OpModeVisionRedFar.TeamPropLocation.Right){
            return 1;
        }
        else if(a == OpModeVisionRedFar.TeamPropLocation.Left){
            return 3;
        }
        else if (a == OpModeVisionRedFar.TeamPropLocation.Middle) {
            return 2;
        }
        else{
            return 0;
        }
    }
    public int convertLocation(OpModeVisionRedClose.TeamPropLocation a){
        if(a == OpModeVisionRedClose.TeamPropLocation.Right){
            return 1;
        }
        else if(a == OpModeVisionRedClose.TeamPropLocation.Left){
            return 3;
        }
        else if (a == OpModeVisionRedClose.TeamPropLocation.Middle) {
            return 2;
        }
        else{
            return 0;
        }
    }
    public int convertLocation(OpModeVisionBlueFar.TeamPropLocation a){
        if(a == OpModeVisionBlueFar.TeamPropLocation.Right){
            return 1;
        }
        else if(a == OpModeVisionBlueFar.TeamPropLocation.Left){
            return 3;
        }
        else if (a == OpModeVisionBlueFar.TeamPropLocation.Middle) {
            return 2;
        }
        else{
            return 0;
        }
    }
    public int convertLocation(OpModeVisionBlueClose.TeamPropLocation a){
        if(a == OpModeVisionBlueClose.TeamPropLocation.Right){
            return 1;
        }
        else if(a == OpModeVisionBlueClose.TeamPropLocation.Left){
            return 3;
        }
        else if (a == OpModeVisionBlueClose.TeamPropLocation.Middle) {
            return 2;
        }
        else{
            return 0;
        }
    }
    public int convertLocation(){
        if(l == org.firstinspires.ftc.teamcode.common.vision.Location.RIGHT){
            return 1;
        }
        else if(l == org.firstinspires.ftc.teamcode.common.vision.Location.LEFT){
            return 3;
        }
        else if (l == org.firstinspires.ftc.teamcode.common.vision.Location.CENTER) {
            return 2;
        }
        else{
            return 0;
        }
    }
    public org.firstinspires.ftc.teamcode.common.centerstage.Location returnLocation(){
        int s = convertLocation();
        switch (s){
            case 1:
                return org.firstinspires.ftc.teamcode.common.centerstage.Location.RIGHT;
            case 2:
                return org.firstinspires.ftc.teamcode.common.centerstage.Location.CENTER;
            case 3:
                return org.firstinspires.ftc.teamcode.common.centerstage.Location.LEFT;
        }
        return org.firstinspires.ftc.teamcode.common.centerstage.Location.UNFOUND;
    }

}
