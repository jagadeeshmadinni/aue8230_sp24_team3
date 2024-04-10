# aue_finals.py is the integrator file
	-- will get cmd_vel and counter when necessary from wall_following, line_following and stop_sign.
	-- will initialize nodes for all.
	-- will make decision to and publish /cmd_vel to the robot
# py file - wall_following(bool startSignal) -> bool startLineFollowing
	-- will return twist to aue_finals
# py file - line_following(bool startLineFollowing) ->bool endProgram
	-- input argument stop_sign detected(yes/no) and distance to stop sign
	-- return I have stopped/not for stop sign
	-- will return twist to aue_finals
# py file - stop_sign_detection (bool startLineFollowing)
	-- return /stop_sign with is there a stop sign and where is it?
	-- end routine once bot is stopped for stop signed
	-- will return twist to aue_finals
