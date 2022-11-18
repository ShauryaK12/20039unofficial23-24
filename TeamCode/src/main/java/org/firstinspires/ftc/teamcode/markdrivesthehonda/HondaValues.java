package org.firstinspires.ftc.teamcode.markdrivesthehonda;

// How to use my HondaValues
// This is divided into 3 parts and all have a letter for each type
// 1. h = Hardware. (also honda) Any values for the robot
// 2. m = Purely messages. Ex: mComplete just gives back "Initialized"
// 3. t = Telemetry Titles. The first arg when calling telemetry
//

public enum HondaValues
{
    //region Hardware

    hFrontLeftWheel("wheelFL"),
    hFrontRightWheel("wheelFR"),

    hBackLeftWheel("wheelBL"),
    hBackRightWheel("wheelBR"),

    //endregion

    //region Purely Messages

    mComplete("Initialized"),
    mStopped("Stopped"),
    mFailed("Failed"),

    //endregion

    // region Telemetry Titles

    tStatus("Robot Status: "),
    tMotors("Motors Status: "),


    //endregion

    ; // ; needed



    // constructor for enums
    final String value;
    HondaValues(String value)
    {
        this.value = value;
    }
    public String getValue() { return this.value; }


}

