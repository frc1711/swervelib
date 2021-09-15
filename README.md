# swerve

This is just a small library in order to help implement swerve drive more easily.
In order to use it, first download the 
[latest release](https://github.com/frc1711/swerve/tree/main/swervelib/build/libs)
and, in your wpilib project, create a new directory named `libs` and place the `jar` inside.
Next, navigate to your project's `build.gradle` file and, under the `dependencies` section,
add the following line:
```
compile name: 'BUILDNAME'
```

Above the `dependencies` section, you should also add the following:

```
repositories {
	flatDir {
		dirs 'libs'
	}
}
```

Replace 'BUILDNAME' with the name of the `.jar` release. From here, you should be ready to import
`frc.team1711.swerve` subpackages in your robot code and use the library. I recommend looking through the
[documentation](https://raw.githack.com/frc1711/swerve/main/swervelib/build/docs/javadoc/index.html).
