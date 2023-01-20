package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Duck;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;
import java.util.ArrayList;


@Autonomous(name="blueSkeletonSM", group="Auton")
public class SkeletonSM extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    BotHardware robot = new BotHardware();

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    private ElapsedTime runtime = new ElapsedTime();

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//Tag ID's of sleeve.
    int pos1 = 1;
    int pos2 = 2;
    int pos3 = 3;
    AprilTagDetection tagOfInterest = null;

    public enum MotorState{
        LVL,
        RESET
    };

    public enum LiftState{
        LIFT_START,
        LIFT_UP_INIT,
        LIFT_DOWN,
        LIFT_UP_REP,
        LIFT_END
    };

    MotorState motorState = MotorState.LVL;
    LiftState liftState = LiftState.LIFT_START;
    //TOWER ARRAY
    int[] towerPos = {20, -90, -247, -342, -470};

    ElapsedTime liftTimer = new ElapsedTime();

    int c = 4;
    int target = 0;
    boolean GoPos = true;
    // LIFT POSITIONS
    public void Update(int target){
//        drive.lift.switchToLevel(target);
        robot.lift.setPower(-1);
        StateUpdate(true, target);
    }

    public void SetTargetPosition(int poz){
        robot.lift.setTargetPosition(poz);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Reset(){
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusy(){return robot.lift.isBusy();}

    public void StateUpdate(boolean IsAuto, int target){
        switch (motorState){
            case LVL:
                Position_Lvl(target);
                break;
            case RESET:
                if(IsAuto)robot.lift.setPower(0);
                Reset();
                break;
        }
        if(!GoPos){
            motorState = MotorState.RESET;
        }
    }

    public void Position_Lvl(int poz2){
        SetTargetPosition(poz2);
        if(!isBusy()){
            GoPos = false;
        }
    }

    @Override
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftTimer.reset();

        ///// TRAJECTORIES /////
        //FORWARD
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(63)
                .build();
        //FORWARD AFTER TURN
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(8)
                .build();
        //BACK THEN TURN
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(6)
                .addTemporalMarker(0, () -> {
                    resetRuntime();
                    drive.R1.setPower(-1.0);
                    drive.L1.setPower(-1.0);
                })
                .build();
        //FORWARD TO CONES
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(44)
                .addTemporalMarker(1, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(1.0);
                })
                .build();
        // BACK OFF THE CONES
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(43)
                .build();
        //FORWARD AND COLLECT
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(6)
                .addTemporalMarker(0, () ->{
                    resetRuntime();
                    drive.L1.setPower(-1.0);
                    drive.R1.setPower(-1.0);

                })
                .build();

        //////APRIL TAGS DETECTION/////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        drive.wrist.setPosition(0.5);
        while (!isStarted() && !isStopRequested())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == pos1 || tag.id == pos2 || tag.id == pos3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

    //////START STATE MACHINE//////
        drive.lift.setPower(-1);

        switch(liftState){
            //START
            case LIFT_START:
                waitForStart();
         //SWITCH STATE
                liftState = LiftState.LIFT_UP_INIT;
                break;

                case LIFT_UP_INIT:
                    //INITIALIZATION
                Update(-2900);
                drive.followTrajectory(traj1);
                drive.turn(Math.toRadians(65));
                drive.followTrajectory(traj2);

                liftState = LiftState.LIFT_DOWN;
                break;
            case LIFT_DOWN:
                //REPEAT FORWARD/BACKWARD TURN
                Update(towerPos[c]);
                drive.followTrajectory(traj3);
                drive.turn(Math.toRadians(-140));
                drive.followTrajectory(traj4);

                c = c-1;
                if(c >= 0){
                    liftState = LiftState.LIFT_UP_REP;
                }else{
                    liftState = LiftState.LIFT_END;
                }
                break;
            case LIFT_UP_REP:
                Update(-2900);
                drive.followTrajectory(traj5);
                drive.turn(Math.toRadians(65));
                drive.followTrajectory(traj6);

                liftState = LiftState.LIFT_DOWN;
                break;
            case LIFT_END:
                Update(-470);
                break;
        }

        if(tagOfInterest.id == pos1){
            //park 1
        }else if(tagOfInterest.id == pos2){
            //park 2
        }else{
            //park 3
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
