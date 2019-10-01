package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=560, gearing=20, maxRPM=315, orientation= Rotation.CW)
@DeviceProperties(xmlTag="GoBilda192GearMotor", name="GoBilda 19.2:1 Motor")
@DistributorInfo(distributor="GoBilda", model="5202-0002-0019", url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-312-rpm-3-3-5v-encoder/")
@ModernRoboticsMotorControllerParams(P=160, I=32, D=112, ratio=25)
public interface GoBilda192GearMotor {
}
