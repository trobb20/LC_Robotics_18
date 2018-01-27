package org.firstinspires.ftc.robotcontroller.external;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

@Autonomous(name="move", group= "Autonomous")
public class move extends LinearOpMode {

    // motor controllers
    DcMotorController driveController;
    DcMotorController armController;

    //servo controllers
    ServoController clawController;

    // motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLift;

    //servos
    Servo topRight;
    Servo topLeft;
    Servo bottomRight;
    Servo bottomLeft;

    private ElapsedTime     runtime = new ElapsedTime();


    @Override


    public void runOpMode() {

        // motor controllers
        driveController = hardwareMap.dcMotorController.get("driveController");
        armController = hardwareMap.dcMotorController.get("armController");
        //servo controllers
        clawController=hardwareMap.servoController.get("clawController");

        //dcMotors
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        //servos
        topRight = hardwareMap.servo.get("topRight");
        topLeft = hardwareMap.servo.get("topLeft");
        bottomRight = hardwareMap.servo.get("bottomRight");
        bottomLeft = hardwareMap.servo.get("bottomLeft");
        //sensors

        //Set all powers to 0
        motorRight.setPower(0);
        motorLeft.setPower(0);
        motorLift.setPower(0);

        //close the claw
        topRight.setPosition(0);
        topLeft.setPosition(1);
        bottomRight.setPosition(1);
        bottomLeft.setPosition(0);

        //swap right direction
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        while(runtime.seconds()<1) {
            motorRight.setPower(1);
            motorLeft.setPower(1);
        }
    }
}
