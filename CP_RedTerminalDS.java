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


@Autonomous(name="CP_RED_Terminal_DS", group="Auton")
public class CP_RedTerminalDS extends LinearOpMode {

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
        SENSOR_CHECK,
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
    double minDSPoleTurn = 25;    double maxDSPoleTurn = 15;
    double minDSPoleFWD = 3;        double maxDSPoleFWD = 4;
    double minDsPoleBWD = 20;        double maxDSPoleBWD = 7;
    double minDSTowerTurn = 100;   double maxDSTowerTurn = 15;
    double minDSTowerFWD = 3;    double maxDSTowerFWD = 10;
    double minDsTowerBWD = 300;        double maxDSTowerBWD = 400;
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
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .build();

        Trajectory trajBack = drive.trajectoryBuilder(new Pose2d())
                .back(4)
                .build();

        //FORWARD AFTER TURN
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5.5)
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

        //BACK THEN TURN
        Trajectory traj7 = drive.trajectoryBuilder(traj2.end())
                .back(11.5)
                .addTemporalMarker(0, () -> {
                    resetRuntime();
                    drive.R1.setPower(-1.0);
                    drive.L1.setPower(-1.0);
                })
                .build();

        Trajectory p1 = drive.trajectoryBuilder(traj7.end())
                .back(27)
                .build();
        Trajectory p2 = drive.trajectoryBuilder(traj7.end())
                .back(5)
                .build();
        Trajectory p3 = drive.trajectoryBuilder(traj7.end())
                .forward(24)
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

        while (opModeIsActive() && !isStopRequested()) {
            Update(target);
            switch(currentState){
                case TRAJ_1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(trajBack);
                        currentState = State.TRAJ_BACK;
                    }
                    break;
                case TRAJ_BACK:
                    target = -2950;
                    telemetry.addData("Currently running: ", currentState);
                    Update(target);
                    if (!drive.isBusy()) {
                        drive.turnAsync(Math.toRadians(-25)); //Turn a bit before searching for pole
                        currentState = State.SENSOR_CHECK;
                    }
                case SENSOR_CHECK:
                    target = -2950;
                    Update(target);
                    telemetry.addData("Currently running: ", currentState);
                    Update(target);
                    if (!drive.isBusy()) {
                        rightTurn(.2, minDSPoleTurn, maxDSPoleTurn);
                        currentState = State.TURN_1;
                    }

                case TURN_1:
                    target = -2950;
                    Update(target);

                    //Once turn is finished, move to traj 2
                    if (!drive.isBusy()) {
                        robot.distancePoleServo.setPosition(sensorUp);
                        drive.followTrajectoryAsync(traj2);
                        currentState = State.TRAJ_2;
                    }
                    break;
                case TRAJ_2:
                    //Once traj 2 is finished (and over pole), drop lift to tower 5, move to traj 3
                    if(!drive.isBusy()){
                        target = -2050;
                        Update(target);
                        drive.followTrajectoryAsync(traj7);
                        currentState = State.TRAJ_3;

                    }
                    break;
                case TRAJ_3:
                    if (!drive.isBusy()) {
                        resetRuntime();
                        drive.turnAsync(Math.toRadians(95));
                        currentState = State.TURN_2;
                    }
                    break;
                case TURN_2:
                    Update(target);

                    if(!drive.isBusy()) {
                        leftTurnTower(.25, minDSTowerTurn, maxDSTowerTurn);
                        currentState = State.PARK;
                    }
                case PARK:
                    if(!drive.isBusy()){
                        robot.distancePoleServo.setPosition(sensorUp);
                        drive.wrist.setPosition(0);
                        drive.L1.setPower(0);
                        drive.R1.setPower(0);
                        target = -100;
                        Update(target);
                    }
                    if(tagOfInterest.id == pos1){
                        //park 1
                        if (!drive.isBusy()) {
                            target = -100;
                            Update(target);
                            drive.turnAsync(Math.toRadians(-20));
                            drive.followTrajectoryAsync(p1);
                            currentState = State.IDLE;
                        }
                    }else if(tagOfInterest.id == pos2){
                        //park 2
                        if (!drive.isBusy()) {
                            target = -100;
                            Update(target);
                            drive.followTrajectoryAsync(p2);
                            currentState = State.IDLE;
                        }
                    }else{
                        //park 3
                        if (!drive.isBusy()) {
                            target = -100;
                            Update(target);
                            drive.turnAsync(Math.toRadians(-20));
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
        robot.turnLeft(power);
        while(robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()){
            telemetry.addData("Currently turning LEFT in: ", currentState);
            telemetry.addData("CM seen: ", robot.distanceTower.getDistance(DistanceUnit.CM));
            Update(target);
            if(robot.lift.getCurrentPosition() < -600) {
                robot.distancePoleServo.setPosition(sensorDown);
            }
        }
        robot.stop();
        Update(target);
    }

    public void leftTurnManual (double time, double power){
        Update(target);
        robot.turnLeft(power);
        resetRuntime();
        while (opModeIsActive() && (runtime.seconds() < time)){
            Update(target);
            if(robot.lift.getCurrentPosition() < -600) {
                robot.distancePoleServo.setPosition(sensorDown);
            }
        }
        robot.stop();
    }

    public void leftTurnTower (double power, double minDS, double maxDS) {
        if(robot.lift.getCurrentPosition() < -600) {
            robot.distancePoleServo.setPosition(sensorDown);
        }
        robot.turnLeft(power);
        while (robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()) {
            telemetry.addData("Currently turning RIGHT in: ", currentState);
            telemetry.addData("CM seen: ", robot.distanceTower.getDistance(DistanceUnit.CM));
            Update(target);
        }
        robot.stop();
        Update(target);
    }

    public void rightTurn (double power, double minDS, double maxDS) {
        resetRuntime();
        while(runtime.milliseconds() < 1000) {
            Update(target);
        }

        robot.turnRight(power);
        while (robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()) {
            telemetry.addData("Currently turning RIGHT in: ", currentState);
            telemetry.addData("CM seen: ", robot.distanceTower.getDistance(DistanceUnit.CM));
            Update(target);
        }
        robot.stop();
        Update(target);
    }

    public void rightTurnTower (double power, double minDS, double maxDS) {
        if(robot.lift.getCurrentPosition() < -600) {
            robot.distancePoleServo.setPosition(sensorDown);
        }
        robot.turnRight(power);
        while (robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()) {
            telemetry.addData("Currently turning RIGHT in: ", currentState);
            Update(target);
        }
        robot.stop();
        Update(target);
    }

    public void rightTurnManual (double time, double power){
        resetRuntime();
        Update(target);
        robot.turnRight(power);
        while (opModeIsActive() && runtime.seconds() < time){
            Update(target);
        }
        robot.stop();
    }

    public void forward (double power, double servoPower, double minDS, double maxDS){
        while(robot.distanceTower.getDistance(DistanceUnit.CM) > minDS && opModeIsActive()){
            //robot.distancePoleServo.setPosition(sensorUp);
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
        //robot.distancePoleServo.setPosition(sensorUp);
        while(robot.distanceTower.getDistance(DistanceUnit.CM) < minDS && opModeIsActive()){
            Update(target);
            if(robot.lift.getCurrentPosition() < -2550 && robot.lift.getCurrentPosition() > -400){
                robot.distancePoleServo.setPosition(sensorUp);
                robot.L1.setPower(servoPower);
                robot.R1.setPower(servoPower);
            }
            if(robot.lift.getCurrentPosition() < -100 && robot.lift.getCurrentPosition() > -400){
                robot.distancePoleServo.setPosition(sensorDown);
                robot.L1.setPower(servoPower);
                robot.R1.setPower(servoPower);
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
