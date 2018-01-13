package org.firstinspires.ftc.robotcontroller.external;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="LcAutoNoah", group= "Autonomous")
public class LcAutoNoah extends LinearOpMode {
    DcMotorController driveController;
    DcMotorController armController;
    ServoController clawController;
    Servo topRight;
    Servo topLeft;
    Servo bottomRight;
    Servo bottomLeft;
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLift;
    ColorSensor colorSensor;
    DeviceInterfaceModule interfaceModule;
    // String clawControl;
    enum BallColor {
        RED, BLUE, NEITHER
    }
    public void runOpMode() {
        interfaceModule = hardwareMap.deviceInterfaceModule.get("interfaceModule");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        driveController = hardwareMap.dcMotorController.get("driveController");
        armController = hardwareMap.dcMotorController.get("armController");
        clawController=hardwareMap.servoController.get("clawController");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        topRight = hardwareMap.servo.get("topRight");
        topLeft = hardwareMap.servo.get("topLeft");
        bottomRight = hardwareMap.servo.get("bottomRight");
        bottomLeft = hardwareMap.servo.get("bottomLeft");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorLift.setPower(0);

        //turn led on
        //colorSensor.enableLed(bLedOn);

        // clawControl = "automatic";
        /*motorRight.setPower(0.5);
        motorLeft.setPower(0.5);*/
        sleep(500);
        // START TEST
        while((checkColor(colorSensor) != BallColor.RED) && (checkColor(colorSensor)!= BallColor.BLUE)) {
            telemetry.addData("Color", "Color: " + checkColor(colorSensor));
            //sleep(500);
        }
        // END TEST
    }
    public static BallColor checkColor(ColorSensor colorSensor) {
        BallColor ballColor;
        if(colorSensor.blue() >= 2 * colorSensor.red()) {
            ballColor = BallColor.BLUE;
        }
        else if((colorSensor.red() >= 2 * colorSensor.blue())){
            ballColor = BallColor.RED;
        }
        else{
            ballColor = BallColor.NEITHER;
        }
        return ballColor;
    }
}
