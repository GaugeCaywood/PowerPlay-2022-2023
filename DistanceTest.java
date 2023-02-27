package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Duck;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.Math;
import java.util.ArrayList;


@TeleOp(name="DS Test", group="Auton")
public class DistanceTest extends LinearOpMode {
public enum State{
    T6,
    T1,
    T2,
    T3,
    T4
};
    State currentState = State.T6;

    BotHardware robot = new BotHardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.distancePoleServo.setPosition(1);

        waitForStart();

        currentState = State.T6;
        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad1.right_bumper){
                currentState = State.T6;
            }
            if(gamepad1.a){         //Test turn left to pole
                currentState = State.T1;
            }
            if(gamepad1.b){   //Test forward to pole
                currentState = State.T2;
            }
            if(gamepad1.x){   //Test turn right to stack
                currentState = State.T3;
            }
            if(gamepad1.y){   //Test forward to stack
                currentState = State.T4;
            }

            if(gamepad1.dpad_up){
                robot.distancePoleServo.setPosition(1);
            }
            if(gamepad1.dpad_down){
                robot.distancePoleServo.setPosition(0.5);
            }

            switch(currentState){
                case T6:
                    robot.stop();
                    telemetry.addData("Servo Position: ", robot.distancePoleServo.getPosition());
                    telemetry.addData("range: ", String.format("%.01f mm", robot.distanceTower.getDistance(DistanceUnit.MM)));
                    telemetry.addData("range: ", String.format("%.01f cm", robot.distanceTower.getDistance(DistanceUnit.CM)));
                    telemetry.addData("range: ", String.format("%.01f m", robot.distanceTower.getDistance(DistanceUnit.METER)));
                    telemetry.addData("range: ", String.format("%.01f in", robot.distanceTower.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    if(gamepad1.dpad_up){
                        robot.distancePoleServo.setPosition(1);
                    }
                    if(gamepad1.dpad_down){
                        robot.distancePoleServo.setPosition(0.5);
                    }
                    break;
                case T1:
                    while(opModeIsActive() && (robot.distanceTower.getDistance(DistanceUnit.CM) > 10)){
                        robot.turnLeft(0.5);
                    }
                    robot.stop();
                    currentState = State.T6;
                    break;
                case T2:
                    while(opModeIsActive() && (robot.distanceTower.getDistance(DistanceUnit.CM) > 1)){
                        robot.forward(0.5);
                    }
                    robot.stop();
                    robot.distancePoleServo.setPosition(0);
                    currentState = State.T6;
                    break;
                case T3:
                    while(opModeIsActive() && (robot.distanceTower.getDistance(DistanceUnit.CM) > 50)){
                        robot.turnRight(0.5);
                    }
                    robot.stop();
                    currentState = State.T6;
                    break;
                case T4:
                    while(opModeIsActive() && (robot.distanceTower.getDistance(DistanceUnit.CM) > 10)){
                        robot.forward(0.5);
                    }
                    robot.stop();
                    robot.distancePoleServo.setPosition(0);
                    currentState = State.T6;
                    break;

            }
        }
    }
}
