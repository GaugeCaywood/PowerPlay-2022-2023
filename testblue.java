/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Duck;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import java.lang.reflect.Method;
import java.util.ArrayList;

@Autonomous(name="testRed", group="Auton")
public class testRed extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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
    //BotHardware robot = new BotHardware();

    @Override
    public void runOpMode()
    {
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        telemetry.addData("Lift Motor", drive.lift.getCurrentPosition() );
        telemetry.update();

        resetRuntime();
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(63)
                //.addTemporalMarker(1, () -> {

              //})
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addTemporalMarker(0, () ->{
                    drive.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.lift.setPower(-0.1);
                })
                .forward(8)
                .addTemporalMarker(5, () ->{
                    drive.lift.setPower(0.1);
                    drive.lift.setTargetPosition(0);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    resetRuntime();
                    //drive.L1.setPower(-1.0);
                    //drive.R1.setPower(1.0);
//                    while (runtime.seconds() < .25){
//
//                    }
//                    drive.L1.setPower(0.00);
//                    drive.R1.setPower(0.0);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(6)
                .build();
        Trajectory traj4;
        traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(44)
                .addTemporalMarker(3, () ->{
            drive.lift.getCurrentPosition();
            drive.lift.setTargetPosition(-470);
            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.lift.setPower(0.50);
//                    while (drive.lift.getCurrentPosition() < 700 && drive.lift.getCurrentPosition() > 650) {
//                    }
//                    drive.lift.setPower(0);
                  })
                .addTemporalMarker(1, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(-1.0);
                    while (runtime.seconds() < 1.5) {
                    }
//                    drive.R1.setPower(0.0);
//                    drive.L1.setPower(0.0);
                })

                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .addDisplacementMarker( (0), () -> {
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-2900);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(1);
//                    while (drive.lift.getCurrentPosition() < -3000 && drive.lift.getCurrentPosition() > -2800){
//                        drive.lift.setPower(0);
//                    }
                })
                .back(43)

                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(2)
                .addTemporalMarker(.1, () ->{
                    resetRuntime();
                    drive.L1.setPower(1.0);
                    drive.R1.setPower(-1.0);

                })
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(2)
                .addTemporalMarker(0.1, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-342);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);

                })
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(44)
                .addTemporalMarker(2, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(-1.0);
//                    while (runtime.seconds() < 1.5) {
//                    }
//                    drive.R1.setPower(0.0);
//                    drive.L1.setPower(0.0);
                })
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .addTemporalMarker( 1, () -> {
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-2900);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(1);
//                    while (drive.lift.getCurrentPosition() < -3000 && drive.lift.getCurrentPosition() > -2800){
//                        drive.lift.setPower(0);
//                    }
                })
                .back(43)

                .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .forward(2)
                .addTemporalMarker(.1, () ->{
                    resetRuntime();
                    drive.L1.setPower(1.0);
                    drive.R1.setPower(-1.0);

                })
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(2)
                .addTemporalMarker(0.1, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-247);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);

                })
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .forward(44)
                .addTemporalMarker(2, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-90);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);
//                    while (drive.lift.getCurrentPosition() < 700 && drive.lift.getCurrentPosition() > 650) {
//                    }
//                    drive.lift.setPower(0);
                })
                .addTemporalMarker(2, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(-1.0);
                    while (runtime.seconds() < 1.5) {
                    }
//                    drive.R1.setPower(0.0);
//                    drive.L1.setPower(0.0);
                })

                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .addTemporalMarker( 2, () -> {
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-2900);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(1);
//                    while (drive.lift.getCurrentPosition() < -3000 && drive.lift.getCurrentPosition() > -2800){
//                        drive.lift.setPower(0);
//                    }
                })
                .back(43)

                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj13.end())
                .forward(2)
                .addTemporalMarker(0.5, () ->{
                    resetRuntime();
                    drive.L1.setPower(1.0);
                    drive.R1.setPower(-1.0);

                })
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end())
                .back(2)
                .addTemporalMarker(0.1, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-0);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);

                })
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end())
                .forward(44)
                .addTemporalMarker(2, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-470);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);
//                    while (drive.lift.getCurrentPosition() < 700 && drive.lift.getCurrentPosition() > 650) {
//                    }
//                    drive.lift.setPower(0);
                })
                .addTemporalMarker(3, () -> {
                    resetRuntime();
                    drive.R1.setPower(1.0);
                    drive.L1.setPower(-1.0);
                    while (runtime.seconds() < 1.5) {
                    }
//                    drive.R1.setPower(0.0);
//                    drive.L1.setPower(0.0);
                })
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end())

                .back(43)
                .addTemporalMarker( 2, () -> {
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-2900);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(1);
//                    while (drive.lift.getCurrentPosition() < -3000 && drive.lift.getCurrentPosition() > -2800){
//                        drive.lift.setPower(0);
//                    }
                })
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(traj17.end())
                .forward(2)
                .addTemporalMarker(0.1, () ->{
                    resetRuntime();
                    drive.L1.setPower(1.0);
                    drive.R1.setPower(-1.0);

                })
                .build();
        Trajectory traj19 = drive.trajectoryBuilder(traj18.end())
                .back(2)
                .addTemporalMarker(0.1, () ->{
                    drive.lift.getCurrentPosition();
                    drive.lift.setTargetPosition(-342);
                    drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.lift.setPower(0.50);

                })
                .build();

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

        /* Actually do something useful */
        //(duration, power, position)
        if(tagOfInterest.id == pos3){
            // left code
//            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            drive.lift.setTargetPosition(-3);
//            drive.lift.setPower(-0.5);
            drive.followTrajectory(traj1);
//            drive.lift.setTargetPosition(-2900);
      //drive.turn(Math.toRadians(90));
      //drive.turn(Math.toRadians(-88));
      //drive.followTrajectory(traj20);
            drive.turn(Math.toRadians(65));
            telemetry.addData("Lift Motor", drive.lift.getCurrentPosition() );
            telemetry.update();
            drive.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.lift.setPower(-0.5);
            drive.lift.setTargetPosition(-2900);
            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
           drive.followTrajectory(traj2);


           //drive.followTrajectory(traj3); OPEN ME
//      drive.turn(Math.toRadians(-140));
//           drive.followTrajectory(traj4);

//            drive.followTrajectory(traj3);
//            drive.turn(Math.toRadians(-105));
//            drive.followTrajectory(traj4);
//            drive.followTrajectory(traj5);
//            drive.turn(Math.toRadians(105));
//            drive.followTrajectory(traj6);
//            drive.followTrajectory(traj7);
//            drive.turn(Math.toRadians(-105));
//            drive.followTrajectory(traj8);
//            drive.followTrajectory(traj9);
//            drive.turn(Math.toRadians(105));
//            drive.followTrajectory(traj10);
//            drive.followTrajectory(traj11);
//            drive.turn(Math.toRadians(-105));
//            drive.followTrajectory(traj12);
//            drive.followTrajectory(traj13);
//            drive.turn(Math.toRadians(105));
//            drive.followTrajectory(traj15);
//            drive.turn(Math.toRadians(-105));
//            drive.followTrajectory(traj16);
//            drive.followTrajectory(traj17);
//            drive.turn(Math.toRadians(105));
//            drive.followTrajectory(traj18);
//            drive.followTrajectory(traj19);

        }else if (tagOfInterest == null || tagOfInterest.id == pos2){
            //middle code
            drive.followTrajectory(traj1);
        }else if (tagOfInterest.id == pos1){
            //right code

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
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

    //    public void StateUpdate(boolean IsAuto){
//        switch (MotorState) {
//            case LVL;
//                ;
//                Position_Lvl(Target);
//                break;
//                case RESET;
//                if(IsAuto)drive.lift.setPower(0);
//                Reset();
//        }
//        if(!GoPos)switchToReset();
//
//    }
//    public void Position_Lvl (int poz2){
//        SetTargetPosition(poz2);
//        if(!isBusy()){
//            GoPos = false;
//        }
//    }
//    public void SetTargetPosition(int poz2){
//        drive.lift.setTargetPosition(poz2);
//        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    public void Reset(){
//        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//    public boolean isBusy(){ return drive.lift.isBusy();}
//    private void forwardT(double duration, double power){
//        //move code
//        robot.fl.setPower(power);
//        robot.fr.setPower(power);
//        robot.bl.setPower(power);
//        robot.br.setPower(power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot.bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void backwardT(double duration,double power){
//        //move code
//        robot.fl.setPower(-power);
//        robot.fr.setPower(-power);
//        robot.bl.setPower(-power);
//        robot.br.setPower(-power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot. bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void strafeLeft(double duration,double power){
//        //move code
//        robot.fl.setPower(-power);
//        robot.fr.setPower(power);
//        robot.bl.setPower(power);
//        robot.br.setPower(-power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot.bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void strafeRight(double duration,double power){
//        //move code
//        robot.fl.setPower(power);
//        robot.fr.setPower(-power);
//        robot.bl.setPower(-power);
//        robot.br.setPower(power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot.bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void turnLeft(double duration,double power){
//        //move code
//        robot.fl.setPower(-power);
//        robot.fr.setPower(power);
//        robot.bl.setPower(-power);
//        robot.br.setPower(power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot.bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void turnRight(double duration,double power){
//        //move code
//        robot.fl.setPower(power);
//        robot.fr.setPower(-power);
//        robot.bl.setPower(power);
//        robot.br.setPower(-power);
//
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.fl.setPower(0);
//        robot.fr.setPower(0);
//        robot.bl.setPower(0);
//        robot.br.setPower(0);
//
//    }
//    private void liftUp(double duration, double power, int position){
//        drive.lift.getCurrentPosition();
//        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.lift.setTargetPosition(position);
//        drive.lift.setPower(-power);
//
//    }
//
//    private void liftDown(double duration, double power, int position){
//        robot.lift.getCurrentPosition();
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setTargetPosition(position);
//        robot.lift.setPower(power);
//        runtime.reset();
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//        robot.lift.setPower(0);
//
//    }
//    private void openClaw(double duration){
//        robot.wrist.setPower(0);
//        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
//        }
//}
//    private  void closeClaw(double duration){
//
//        robot.wrist.setPower(1);
//    }
}


