/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "SensorMRColorSamTest", group = "Autonomous")
public class SensorMRColorSamTest extends LinearOpMode {

    DeviceInterfaceModule interfaceModule;
    ColorSensor colorSensor;
    Servo armServo1;
    Servo armServo2;


  @Override
  public void runOpMode() {

      armServo1 = hardwareMap.servo.get("armServo1");
      armServo2 = hardwareMap.servo.get("armServo2");
      interfaceModule = hardwareMap.deviceInterfaceModule.get("interfaceModule");
      //sensors
      colorSensor = hardwareMap.colorSensor.get("colorSensor");

      //set servos to initial positions
      armServo1.setPosition(0.75);
      armServo2.setPosition(0.5);

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

        colorSensor.enableLed(bLedOn);

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

      // send the info back to driver station using telemetry function.
      if(colorSensor.red()>colorSensor.blue()){
        telemetry.addLine("Red");
      }
      if(colorSensor.blue()>colorSensor.red()){
        telemetry.addLine("Blue");
      }
      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
            //relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));

            int hit = 0;

            armServo1.setPosition(0.1);


            while (hit == 0) {

                double blue = colorSensor.blue();
                double red = colorSensor.red();

                telemetry.addData("Blue:", blue);
                telemetry.addData("Red:", red);
                telemetry.update();

               if (red >= 5) {

                    hitLeft();
                    hit=1;

                }
                if (blue >= 5) {

                    hitRight();
                    hit=1;

                }

            }



        }

        public void hitLeft() {

            armServo2.setPosition(0);
            sleep(3000);
            //set servos to initial positions
            armServo1.setPosition(0.75);
            armServo2.setPosition(0.5);
            stop();

        }

        public void hitRight() {

            armServo2.setPosition(1);
            sleep(3000);
            //set servos to initial positions
            armServo1.setPosition(0.75);
            armServo2.setPosition(0.5);
            stop();
        }

      });


      bLedOn = !bLedOn;
      colorSensor.enableLed(bLedOn);
    }

    // Set the panel back to the default color
    relativeLayout.post(new Runnable() {
      public void run() {
        relativeLayout.setBackgroundColor(Color.WHITE);
      }
    });
  }
}
