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

package org.firstinspires.ftc.teamcode.OpenCV;

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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import java.lang.reflect.Method;
import java.util.ArrayList;

@Autonomous(name="TR1B2", group="Auton")
public class tr1b2 extends LinearOpMode
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

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//Tag ID's of sleeve.
    int pos1 = 1;
    int pos2 = 2;
    int pos3 = 3;
    AprilTagDetection tagOfInterest = null;
    //BotHardware robot = new BotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    BotHardware robot = new BotHardware();
    @Override
    public void runOpMode()
    {
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
        while (!isStarted() && !isStopRequested())
        {
            robot.init(hardwareMap);

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
        strafeLeft(1.75,.5);
        strafeRight(1.55,.5);
        if(tagOfInterest.id == pos1){
            // left code
            forwardT(1.5,0.5);
            backwardT(.25, .5);
            strafeLeft(1.95, 0.5);

        }else if (tagOfInterest == null || tagOfInterest.id == pos2){
            //middle code

            forwardT(1.25, 0.5);
        }else if (tagOfInterest.id == pos3){
            //right code
            forwardT(1.4, 0.5);
            strafeRight(1.75, 0.5);
            forwardT(.25, .5);
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
    private void forwardT(double duration, double power){
        //move code
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

    }
    private void backwardT(double duration,double power){
        //move code
        robot.fl.setPower(-power);
        robot.fr.setPower(-power);
        robot.bl.setPower(-power);
        robot.br.setPower(-power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot. bl.setPower(0);
        robot.br.setPower(0);

    }
    private void strafeLeft(double duration,double power){
        //move code
        robot.fl.setPower(-power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(-power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

    }
    private void strafeRight(double duration,double power){
        //move code
        robot.fl.setPower(power);
        robot.fr.setPower(-power);
        robot.bl.setPower(-power);
        robot.br.setPower(power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

    }
    private void turnLeft(double duration,double power){
        //move code
        robot.fl.setPower(-power);
        robot.fr.setPower(power);
        robot.bl.setPower(-power);
        robot.br.setPower(power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

    }
    private void turnRight(double duration,double power){
        //move code
        robot.fl.setPower(power);
        robot.fr.setPower(-power);
        robot.bl.setPower(power);
        robot.br.setPower(-power);

        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);

    }
    private void liftUp(double duration, double power, int position){
        robot.lift.getCurrentPosition();
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition(position);
        robot.lift.setPower(-power);
        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.lift.setPower(0);

    }

    private void liftDown(double duration, double power, int position){
        robot.lift.getCurrentPosition();
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setTargetPosition(position);
        robot.lift.setPower(power);
        runtime.reset();
        while ((runtime.seconds() < duration ) && (opModeIsActive())) {
        }
        robot.lift.setPower(0);

    }
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

