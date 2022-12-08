package org.firstinspires.ftc.teamcode.OpenCV;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Last Edited 10/27/2022 6:59PM MST

public class BotHardware
{


    /* Public OpMode members. */
    //MOTOR NULE DECLARATION
    public DcMotor  fr   = null;
    public DcMotor  br   = null;
    public DcMotor  fl   = null;
    public DcMotor  bl   = null;
    // public DcMotor Harvest  = null;
    // public DcMotor duckie = null;
    public DcMotor lift = null;

    BNO055IMU imu;
    // public DcMotor    magnetArm = null;


    //SENSOR DECLARATION
    // public DistanceSensor sensor_range = null;
    // public DistanceSensor Locator = null;


    //SERVO DECLARATION
    // public Servo    bucket  = null;
    //public Servo    Break = null;
    // public Servo    donkey  = null;
    public Servo    wrist = null;
    public Servo    wrist2 = null;
    //   public Servo    hand  = null;

    //HARVESTER SERVOS
    public CRServo    L1  = null;
//    public CRServo    L2  = null;
    public CRServo    R1  = null;
//    public CRServo    R2  = null;


    /* local OpMode members. */
    //DECLARING HARDWARE MAP AND A TIME SYSTEM
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public BotHardware(){

    }


    /* Initialize standard Hardware interfaces */
    //SAYS IT CAN BE ACCESSED or transported
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fr  = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        fl    = hwMap.get(DcMotor.class, "fl");
        bl  = hwMap.get(DcMotor.class, "bl");
        // Harvest = hwMap.get(DcMotor.class, "Harvest");
        // duckie = hwMap.get(DcMotor.class, "duckie");
        lift = hwMap.get(DcMotor.class, "lift");
        // magnetArm = hwMap.get(DcMotor.class,"magnetArm");

        //SETING MOTOR DIRECTIONS
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Harvest.setDirection(DcMotor.Direction.FORWARD);
        // duckie.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        //magnetArm.setDirection(CRServo.Direction.FORWARD);



        // Set all MOTOR zero power
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        // Harvest.setPower(0);
        // duckie.setPower(0);
        lift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Harvest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // duckie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //magnetArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // // Define and initialize ALL installed servos.
        // bucket  = hwMap.get(Servo.class, "bucket");
        // donkey = hwMap.get(Servo.class, "donkey");
        //Break  = hwMap.get(Servo.class, "Break");
        //  wrist2 = hwMap.get(Servo.class, "wrist2");
        wrist  = hwMap.get(Servo.class, "wrist");
//        hand  = hwMap.get(Servo.class, "hand");

        //HARVESTER SERVOS
        L1  = hwMap.get(CRServo.class, "L1");
//        L2  = hwMap.get(CRServo.class, "L2");
        R1  = hwMap.get(CRServo.class, "R1");
//        R2  = hwMap.get(CRServo.class, "R2");

        wrist.setPosition(0);
        L1.setPower(0);
        R1.setPower(0);
    }

    //SET MOTOR SPEEDS AND DIRECTIONS USUALLY WITH MATH ABS
    public void forward(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void backward(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void left(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void right(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void turnLeft(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void turnRight(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }
    public void BackwardLeft (double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(-Math.abs(speed));
    }
    // SET OTHER MOTOR POWERS OR DIRECTIONS
    // public void intake(double speed){
    //     Harvest.setPower(Math.abs(speed));
    //     }

    // public void intake2(double speed){
    //     duckie.setPower(speed);
    //     }

    // public void outtake(double speed){
    //     Harvest.setPower(-Math.abs(speed));
    //     }

    // public void bucketSet(double position){
    //     bucket.setPosition(position);
    //     }

    // public void donkeySet(double position){
    //     donkey.setPosition(position);
    //     }

    // public void liftSet(double RUN_TO_POSITION){
    //     lift.setTargetPosition(setTargetPosition);
    //     }
//
//    public void wristSet(double position){
//        wrist.setPower(position);
//    }
//    public void wrist2Set(double position){
//        wrist2.setPosition(position);
//    }

//    public void handSet(double position){
//        hand.setPosition(position);
//    }
//
}
