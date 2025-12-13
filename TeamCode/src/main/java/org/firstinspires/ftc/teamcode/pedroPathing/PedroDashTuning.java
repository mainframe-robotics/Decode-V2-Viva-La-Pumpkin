package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

@Config
public class PedroDashTuning {
    public static PIDFCoefficients translational = new PIDFCoefficients(
            0.1,
            0,
            0.01,
            0.02
    );
    public static PIDFCoefficients secondaryTranslational = new PIDFCoefficients(
            0.15,
            0,
            0.05,
            0.0005
    );

    public static PIDFCoefficients heading = new PIDFCoefficients(
            5,
            0,
            0.5,
            0.03
    );
    public static PIDFCoefficients secondaryHeading = new PIDFCoefficients(
            3,
            0,
            0.1,
            0.03
    );

    public static FilteredPIDFCoefficients drive = new FilteredPIDFCoefficients(
            0.05,
            0,
            0.0001,
            0.6,
            0.01
    );
    public static FilteredPIDFCoefficients secondaryDrive = new FilteredPIDFCoefficients(
            0.03,
            0,
            0.0005,
            0.6,
            0.01
    );
}
