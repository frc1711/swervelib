# Implementing Swervelib

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

Replace 'BUILDNAME' with the name of the `.jar` release.

# Documentation
Documentation can be viewed online [here](https://raw.githack.com/frc1711/swerve/main/swervelib/build/docs/javadoc/index.html).

# Versioning
With the exception of very early versions of swervelib, versions will be named by v(version)-(wpilib-version).
For example, v2.0-2022.1.1 would be swervelib v2.0, built for wpilib version 2022.1.1.