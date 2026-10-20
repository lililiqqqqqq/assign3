"""Measured fingertip positions for the real keyboard.

All positions are expressed in the robot base frame, in metres.

To calibrate manually:
1. Run robot_node in read-only mode (enable_motors:=false).
2. Run forward_kinematics and read /fk_pose.
3. Manually move the fixed fingertip onto each real key P, O, L.
4. Update the positions below with the measured [x, y, z] from /fk_pose.
5. Measure HOME roughly 5-10 cm above the centre of P/O/L and fill HOME_POSITION.

The values below are the user's measured fingertip positions, in the robot base frame,
in metres. Do not replace them with guessed or simulator coordinates.
"""


REAL_KEY_POSITIONS = {
    "p": [0.2903514547578454, -0.0029286251096077125, -0.03741459074503575],
    "o": [0.2808102480757531, 0.023493049478388025, -0.03969896457931049],
    "l": [0.2584487855595578, 0.01518369349129052, -0.042968220927180334],
}

HOME_POSITION = [
    0.24023776556069998,
    0.025209053343286013,
    -0.014006336216451948,
]
