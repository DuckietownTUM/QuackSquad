import React from "react"

const StatusBar = ({ros, state, error}) => {
	let getRobotStateColor = () => {
		let color

		switch (state) {
			case "IDLE_MODE":
				color = "text-blue-500"
				break;
			case "RECOVERY_MODE":
				color = "text-orange-500"
				break;
			case "LANE_FOLLOWING":
				color = "text-yellow-500"
				break;
			case "NORMAL_JOYSTICK_CONTROL":
				color = "text-green-500"
				break;
			default:
				color = "text-black"
				break;
		}

		return color
	}

	let rosStatus = () => {
		if (error || ros === null || !ros.isConnected)
			return false
	}

	return (
		<div className="z-50 flex justify-center items-center p-4 bg-gray-200 shadow-lg rounded-lg fixed top-[0] w-full">	
			<p className="text-lg mx-4">Status: <span className={`font-semibold ${rosStatus() ? "text-green-600" : "text-red-600"}`}>{error ? "Ros Error" : (rosStatus() ? "Connected" : "Disconnected")}</span></p>
			<p className="text-lg mx-4">State: <span className={`font-semibold ${getRobotStateColor()}`}>{state ? state : "--"}</span></p>
		</div>
	)
}

export default StatusBar