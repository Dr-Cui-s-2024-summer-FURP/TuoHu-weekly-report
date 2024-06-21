# Resolving PX4 Simulator Time Synchronization Issue

**Issue Description:** The system repeatedly displays time synchronization warnings and signals of time synchronization completion.

**Cause of the Issue:** The uXRCE-DDS client has its own timer, which conflicts with the simulation time of gazebo-classic.

**Solution:** Open QGroundControl:

1. Click on the upper right corner of the interface.
2. Select "Vehicle Settings".
3. In the left menu bar, choose "Parameter Settings".
4. Enter [**UXRCE_DDS_SYNCT**](https://docs.px4.io/main/zh/advanced_config/parameter_reference.html#UXRCE_DDS_SYNCT) to find the timer for the [**UXRCE_DDS**](https://docs.px4.io/main/zh/advanced_config/parameter_reference.html#UXRCE_DDS_SYNCT) client.
5. Change "enable" to "disable".
