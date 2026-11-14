Location of external libs

ftclib-core-2.1.1.aar
  FTCLib 2.1.1 (MIT License, https://github.com/FTCLib/FTCLib), copied from Maven Central
  (org.ftclib.ftclib:core:2.1.1). Kept here so the build works offline on the robot's Wi-Fi.
  TeamCode uses only HolonomicOdometry and the geometry classes, which depend on nothing
  outside the JDK. If you start using other FTCLib classes, check whether they need EJML,
  androidx core-ktx, or Kotlin; those are not bundled.
