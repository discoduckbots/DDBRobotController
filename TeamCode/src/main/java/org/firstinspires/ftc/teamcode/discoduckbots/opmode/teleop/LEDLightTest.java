/* Copyright (c) 2017-2020 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.discoduckbots.opmode.teleop;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.discoduckbots.hardware.PixelDetector;

@TeleOp(name = "LED Light Test", group = "Sensor")
@Disabled
public class LEDLightTest extends LinearOpMode {

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  RevBlinkinLedDriver lights;
  PixelDetector detector;

  @Override public void runOpMode() {

    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];
    lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);

    waitForStart();


    while (opModeIsActive()){

      if (gamepad1.cross){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
      }

      if (gamepad1.square){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
      }

      if (gamepad1.circle){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
      }

      if (gamepad1.triangle){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
      }

      if (gamepad1.dpad_up){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
      }

      if (gamepad1.dpad_down){
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
      }

    }
  }
}
