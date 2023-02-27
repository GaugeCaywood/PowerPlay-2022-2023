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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.Math;
import java.util.ArrayList;


@Autonomous(name="1CP_BlueTerminalDS", group="Auton")
public class CP_BlueTerminalDS extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    BotHardware robot = new BotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
//    private ElapsedTime runtime = new ElapsedTime();

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

    public enum State{
        TRAJ_1,
        TRAJ_BACK,
        TURN_1,
        TRAJ_2,
        TRAJ_3,
        TURN_2,
        TRAJ_4,
        TRAJ_5,
        TURN_3,
        TRAJ_6,
        TRAJ_7,
        TURN_4,
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
    double minDSPoleTurn = 30;    double maxDSPoleTurn = 15;
    double minDSPoleFWD = 3;        double maxDSPoleFWD = 4;
    double minDsPoleBWD = 20;        double maxDSPoleBWD = 7;
    double minDSTowerTurn = 69.6;   double maxDSTowerTurn = 15;
    double minDSTowerFWD = 3;    double maxDSTowerFWD = 10;
    double minDsTowerBWD = 300;        double maxDSTowerBWD = 400;
    //Lift variables
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
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .build();

        Trajectory trajBack = drive.trajectoryBuilder(new Pose2d())
                .back(4)
                .build();

        //FORWARD AFTER TURN
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(6.5)
                .build();

        //BACK THEN TURN
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(14)
                .addTemporalMarker(0, () -> {
                    resetRuntime();
                    drive.R1.setPower(-1.0);
                    drive.L1.setPower(-1.0);
                })
                .build();
        //FORWARD TO CONES
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(20)
                .addTemporalMarker(1, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(1.0);
                })
                .build();
        // BACK OFF THE CONES
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(24.25)
                .addTemporalMarker(0, () -> {
                    resetRuntime();
                    drive.R1.setPower(0);
                    drive.L1.setPower(0);
                })
                .build();
        //FORWARD AND COLLECT
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(8)
                .build();

        //BACK THEN TURN
        Trajectory traj7 = drive.trajectoryBuilder(traj2.end())
                .back(11)
                .addTemporalMarker(0, () -> {
                    resetRuntime();
                    drive.R1.setPower(-1.0);
                    drive.L1.setPower(-1.0);
                })
                .build();

        Trajectory p1 = drive.trajectoryBuilder(traj6.end())
                .back(25)
                .build();
        Trajectory p2 = drive.trajectoryBuilder(traj6.end())
                .back(4)
                .build();
        Trajectory p3 = drive.trajectoryBuilder(traj6.end())
                .forward(20)
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

        waitForStart();

        if (isStopRequested()) return;
        //////START STATE MACHINE//////

        //start moving
        currentState = State.TRAJ_1;
        robot.distancePoleServo.setPosition(sensorUp);
        drive.followTrajectoryAsync(traj1);
        // target = 0;

        while (opModeIsActive() && !isStopRequested()) {
            Update(target);
            switch(currentState){
                case TRAJ_1:
                    //Once turn is finished (not busy), move to turn 1
                    //target = -2900;
                    //Update(target);

                    if (!drive.isBusy()) {
                        currentState = State.TRAJ_BACK;
                        drive.followTrajectoryAsync(trajBack);
                    }
                    break;
                case TRAJ_BACK:
                    target = -2950;
                    //robot.distancePoleServo.setPosition(sensorUp);
                    Update(target);
                    if (!drive.isBusy()) {
                       // leftturn(0.3, 100,1000);

                        leftTurn(.3, minDSPoleTurn, maxDSPoleTurn);
                        currentState = State.TURN_1;

                       // drive.turnAsync(Math.toRadians(59));
                    }
                case TURN_1:
                    target = -2950;
                    Update(target);

                    //Once turn is finished, move to traj 2
                    if (!drive.isBusy()) {
                        forward(0.5, 0, minDSPoleFWD, maxDSPoleFWD);
                        currentState = State.TRAJ_2;

                        //drive.followTrajectoryAsync(traj2);
                    }
                    break;
                case TRAJ_2:
                    //Once traj 2 is finished (and over pole), drop lift to tower 5, move to traj 3
                    target = -2650;
                    Update(target);
                    robot.distancePoleServo.setPosition(sensorDown);
                    if(!drive.isBusy()){
                        back(.5,minDsPoleBWD, maxDSPoleBWD, 0);

                        currentState = State.TRAJ_3;

                        //drive.followTrajectoryAsync(traj7);

                    }
                    break;
               case TRAJ_3:
                    //Once traj 3 is finished, move to turn 2

                    if (!drive.isBusy()) {
                        rightTurn(.3,minDSTowerTurn,maxDSTowerTurn);
                        currentState = State.PARK;
                        //drive.turnAsync(Math.toRadians(-122));

                    }
                    break;
                case TURN_2:
                    //Once turn 2 is finished, move to traj 4 (forward into tower)
                    target = towerPos[c];
                    Update(target);

                    if(!drive.isBusy()){
                        c = c-1; //also drop tower counter so it'll be correct next loop
                        forward(.5, 1, minDSTowerFWD, maxDSTowerFWD);
                        slowForward(0.5, 1, .75);
                        currentState = State.TRAJ_4;
                        //drive.followTrajectoryAsync(traj4);
                    }
                    break;
                case TRAJ_4:
                    //Once traj 4 is finished, lift to high level and move to traj 5 (this should lift early enough to not tip the tower but we may need a slight wait)
                    if(!drive.isBusy()){
                        target = -2950;
                        back(1, minDsTowerBWD, maxDSTowerBWD, 0 );
                       // robot.distancePoleServo.setPosition(sensorUp);
                        currentState = State.TRAJ_5;
                        //drive.followTrajectoryAsync(traj5);
                    }
                    break;
                case TRAJ_5:
                    //Once traj 5 is finished, move to turn 3
                    target = -2950;

                    if (!drive.isBusy()) {
                        leftTurn(.5,minDSPoleTurn,maxDSPoleTurn);
                        currentState = State.TURN_3;
                       // drive.turnAsync(Math.toRadians(115));
                    }
                    break;
                case TURN_3:
                    //Once turn 3 is finished, move to traj 6
                    target = -2950;

                    if (!drive.isBusy()) {
                        currentState = State.TRAJ_6;
                        forward(1, 0, minDSPoleFWD,maxDSTowerFWD);
                        //drive.followTrajectoryAsync(traj6);
                    }
                    break;
                case TRAJ_6:
                    //Once traj 6 is finished, check if we still have cones on the tower (c >= 0).
                    //If so, drop to next tower pos and repeat up to Traj2 (which goes into traj3/drop and back from pole)
                    if(!drive.isBusy()){
                        target = towerPos[c];
                        currentState = State.TRAJ_7;
                        back(1,minDsPoleBWD,maxDSPoleBWD,-1);
                        //drive.followTrajectoryAsync(traj7);
                    }
                    break;
                case TRAJ_7:
                    target = towerPos[c];
                    if (!drive.isBusy()) {
                        rightTurn(1,minDSTowerTurn,maxDSPoleTurn);
                        currentState = State.PARK;
                        //drive.turnAsync(Math.toRadians(-119));
                    }
                case PARK:
                    if(!drive.isBusy()){
                        drive.wrist.setPosition(0);
                        drive.L1.setPower(0);
                        drive.R1.setPower(0);
                    }
                    if(tagOfInterest.id == pos1){
                        //park 1
                        if (!drive.isBusy()) {
                            drive.turnAsync(Math.toRadians(20));
                            drive.followTrajectoryAsync(p1);
                            currentState = State.IDLE;
                        }
                    }else if(tagOfInterest.id == pos2){
                        //park 2
                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(p2);
                            currentState = State.IDLE;
                        }
                    }else{
                        //park 3
                        if (!drive.isBusy()) {
                            drive.turnAsync(Math.toRadians(20));
                            drive.followTrajectoryAsync(p3);
                            currentState = State.IDLE;
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

    public void leftTurn (double power, double minDS, double maxDS){
//        resetRuntime();
        robot.turnLeft(power);
        while(robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()){
            Update(target);
            if(robot.lift.getCurrentPosition() < -600) {
                robot.distancePoleServo.setPosition(sensorDown);
            }
        }
        robot.stop();
        Update(target);
    }
    public void rightTurn (double power, double minDS, double maxDS){
//        resetRuntime();
        robot.turnRight(power);
        while(robot.distanceTower.getDistance(DistanceUnit.CM ) > minDS && opModeIsActive()){
            Update(target);
        }
        robot.stop();
        Update(target);
    }

    public void forward (double power, double servoPower, double minDS, double maxDS){
        while(robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()){
            robot.distancePoleServo.setPosition(sensorUp);
            Update(target);
            if(robot.lift.getCurrentPosition() < -600) {
                robot.distancePoleServo.setPosition(sensorDown);
                robot.forward(power);
                robot.L1.setPower(servoPower);
                robot.R1.setPower(servoPower);
            }
        }
        robot.stop();
        robot.L1.setPower(0);
        robot.R1.setPower(0);
        Update(target);
    }
    public void back (double power, double minDS, double maxDS, double servoPower){
        robot.backward(power);
        robot.distancePoleServo.setPosition(servoPower);
        while(robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()){
            Update(target);
            if(robot.lift.getCurrentPosition() > -1500 && robot.lift.getCurrentPosition() < -400){
                robot.distancePoleServo.setPosition(sensorUp);
            }
            if(robot.lift.getCurrentPosition() < -100 && robot.lift.getCurrentPosition() > -400){
                robot.distancePoleServo.setPosition(sensorDown);
            }
        }
        robot.stop();
        Update(target);
        robot.L1.setPower(0);
        robot.R1.setPower(0);
    }
    public void slowForward(double power, double servoPower, double time){
        resetRuntime();
        Update(target);
        robot.forward(power);
        robot.L1.setPower(servoPower);
        robot.R1.setPower(servoPower);
        while(opModeIsActive() && runtime.seconds() < time){
            Update(target);
        }
        robot.stop();
        robot.L1.setPower(0);
        robot.R1.setPower(0);
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
