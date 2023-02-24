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


@Autonomous(name="DS Test", group="Auton")
public class DistanceTest extends LinearOpMode {

  BotHardware robot = new BotHardware();
  int test = 0;

  @Override
  public void runOpMode(){
    robot.init(hardwareMap);
    robot.dsArm.setPosition(1);
              
    while (opModeIsActive() && !isStopRequested()) {

      if(gamepad1.a){         //Test turn left to pole
        test = 1;
      }else if(gamepad1.b){   //Test forward to pole
        test = 2;
      }else if(gamepad1.x){   //Test turn right to stack
        test = 3;
      }else if(gamepad1.y){   //Test forward to stack
        test = 4;
      }

      Switch(test){
        case 0:
            telemetry.addData("range: ", String.format("%.01f mm", ds.getDistance(DistanceUnit.MM)));
            telemetry.addData("range: ", String.format("%.01f cm", ds.getDistance(DistanceUnit.CM)));
            telemetry.addData("range: ", String.format("%.01f m", ds.getDistance(DistanceUnit.METER)));
            telemetry.addData("range: ", String.format("%.01f in", ds.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        case 1:
          while(ds.getDistance(DistanceUnit.CM) > 10){
            robot.turnLeft(0.5);
          }
            robot.bl.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.fr.setPower(0);
            test = 0;
          break;
        case 2:
         while(ds.getDistance(DistanceUnit.CM) > 1){
            robot.forward(0.5);
          }
            robot.bl.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.dsArm.setPosition(0);
            test = 0;
          break;
        case 3:
         while(ds.getDistance(DistanceUnit.CM) > 50){
            robot.turnRight(0.5);
          }
            robot.bl.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.fr.setPower(0);
            test = 0;
          break;
        case 4:
         while(ds.getDistance(DistanceUnit.CM) > 10){
            robot.forward(0.5);
          }
            robot.bl.setPower(0);
            robot.fl.setPower(0);
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.dsArm.setPosition(0);
            test = 0;
          break;

      }
    }
  }
}
