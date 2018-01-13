package org.firstinspires.ftc.robotcontroller.external;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by NoahGSimon on 12/19/17.
 */

@Autonomous(name="LcColorCheck", group= "Autonomous")



public class LcCheckColor {

    enum BallColor {
        RED, BLUE;
    }
    public static BallColor checkColor(ColorSensor colorSensor) {
        BallColor ballColor;
        if(colorSensor.blue() >= colorSensor.red()) {
            ballColor = BallColor.BLUE;
        }
        else {
            ballColor = BallColor.RED;
        }
        return ballColor;
    }
}
