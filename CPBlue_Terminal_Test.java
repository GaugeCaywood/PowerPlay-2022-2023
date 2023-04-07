package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Duck;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.Math;
import java.util.ArrayList;


@Autonomous(name="1CP_Blue_Terminal_Odo", group="Auton")
public class CPBlue_Terminal_Test extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    BotHardware robot = new BotHardware();
    public ElapsedTime runtime = new ElapsedTime();
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
////    private ElapsedTime runtime = new ElapsedTime();

    //    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
////Tag ID's of sleeve.
    int pos1 = 1;
    int pos2 = 2;
    int pos3 = 3;
    AprilTagDetection tagOfInterest = null;

    public enum MotorState{
        LVL,
        RESET
    };

    public enum State{
        START,
        COLLECT,
        PARK,
        IDLE
    };

    MotorState motorState = MotorState.LVL;
    State currentState = State.IDLE;

    //TOWER ARRAY
    int[] towerPos = {20, -90, -247, -342, -450};

    ElapsedTime liftTimer = new ElapsedTime();

    //Sensor variables
    double sensorUp = 1;            double sensorDown = 0.5;
    double minDSPoleTurn = 25;    double maxDSPoleTurn = 15;
    double minDSTowerTurn = 100;   double maxDSTowerTurn = 15;

    //Lift variables
    int c = 4;
    int target = 0;
    boolean GoPos = true;

    // LIFT POSITIONS
    public void Update(int target){
//        drive.lift.switchToLevel(target);
        telemetry.update();
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
            //GoPos = false;
        }
    }

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftTimer.reset();

        ///// TRAJECTORIES /////
        //FORWARD
//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .forward(60)
//                .build();
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-35.43, -64.91, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-25.37, -30.63), Math.toRadians(56.31))
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
        robot.wrist.setPosition(0.5);
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

        waitForStart();

        if (isStopRequested()) return;
        //////START STATE MACHINE//////

        //start moving

        currentState = State.COLLECT;
        while (opModeIsActive() && !isStopRequested()) {
            Update(target);

            switch(currentState){
                case COLLECT:
                    Update(target);
                    drive.followTrajectorySequence(untitled0);
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                case PARK:
                    if(!drive.isBusy()){
                       // robot.distancePoleServo.setPosition(sensorUp);

                    }
                    if(tagOfInterest.id == pos1){
                        //park 1
                        if (!drive.isBusy()) {

                        }
                    }else if(tagOfInterest.id == pos2){
                        //park 2
                        if (!drive.isBusy()) {

                        }
                    }else{
                        //park 3
                        if (!drive.isBusy()) {

                        }
                    }
                    break;
                case IDLE:
                    if (!drive.isBusy()) {
                        // end, do nothing
                    }
                    break;
            }

            Update(target);
            drive.update();
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