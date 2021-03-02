# swerve

This is just a small library in order to help implement swerve drive more easily in the future.
In order to use it, download the repository's
[swerve directory](https://downgit.github.io/#/home?url=https://github.com/frc1711/swerve/tree/main/swerve)
and, in your wpilib project, move it into the src/main/java directory. From there, you need to
create a subclass of the abstract `swerve.SwerveWheel` subsystem and implement all of its abstract
methods. It's very important that these methods exactly match their descriptions in the javadocs.
Once that is done, you can use this module class along with `swerve.SwerveDrive` to create a
functional swerve drive. I recommend looking through the
[documentation](https://raw.githack.com/frc1711/swerve/main/javadocs/swerve/package-summary.html).