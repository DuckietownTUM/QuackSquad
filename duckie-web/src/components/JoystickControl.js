import React, { useState, useEffect } from "react"
import { Joystick } from "react-joystick-component"
import ROSLIB from "roslib"

const JoystickControl = ({ros, state, isEStopOn}) => {
	const [pubJoy, setPubJoy] = useState(null)

	useEffect(() => {
		if (!ros)
			return
		
		let pubJoy = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/joy",
			messageType: "sensor_msgs/Joy",
		})
		setPubJoy(pubJoy)
	}, [ros])

	const sendJoyMessage = (axes, buttons) => {
		if (pubJoy) {
			const message = new ROSLIB.Message({
				axes,
				buttons,
			})
			pubJoy.publish(message)
		}
	}
	
	const handleMove = (event) => {
		const x = event.y < 0 ? event.x : -event.x || 0
		const y = event.y || 0
		sendJoyMessage([0, y, 0, x], Array(15).fill(0))
	}

	const handleStop = () => {
		sendJoyMessage([0, 0, 0, 0], Array(15).fill(0))
	}

	const startLineFollowing = () => {
		sendJoyMessage([0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
	}

	const toggleEStop = () => {
		sendJoyMessage([0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
	}

	useEffect(() => {
		let pressedKeys = new Set()

		let updateAxes = () => {
			let axes = [0, 0, 0, 0]

			if (pressedKeys.has("ArrowUp")) axes[1] = 1  // Forward
			if (pressedKeys.has("ArrowDown")) axes[1] = -1 // Backward
			if (pressedKeys.has("ArrowLeft")) axes[3] = 1 // Left
			if (pressedKeys.has("ArrowRight")) axes[3] = -1 // Right

			sendJoyMessage(axes, Array(15).fill(0))
		}

		let handleKeyDown = (event) => {
			if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(event.key)) {
				pressedKeys.add(event.key)
				updateAxes()
			}

			// Check for Ctrl + Shift + /
			if (event.ctrlKey && event.shiftKey && event.key === "/") {
				let buttons = Array(15).fill(0)
				buttons[6] = 1
				sendJoyMessage([0, 0, 0, 0], buttons)
			}
		}

		let handleKeyUp = (event) => {
			pressedKeys.delete(event.key)
			updateAxes()
		}

		window.addEventListener("keydown", handleKeyDown)
		window.addEventListener("keyup", handleKeyUp)

		return () => {
			window.removeEventListener("keydown", handleKeyDown)
			window.removeEventListener("keyup", handleKeyUp)
		}
	})
	

	return (
		<div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-1">🎮 Duckiebot Joystick</h1>
			<div className="mb-4">
				<p className="text-sm"> Use the Joystick or the Arrow keys</p>
				<p className={`text-sm ${["IDLE_MODE", "NORMAL_JOYSTICK_CONTROL"].includes(state) ? "block" : "hidden"}`}>Ctrl+Shift+/ : Toggle Joystick</p>
			</div>
			<Joystick className="flex flex-col gap-2 mt-4" size={150} baseColor="#ddd" stickColor="#999" move={handleMove} stop={handleStop} disabled={isEStopOn || ["IDLE_MODE", "LANE_FOLLOWING"].includes(state)} />

			<div className="flex flex-col gap-2 mt-4">
				<button id="startBtn" onClick={startLineFollowing} disabled={isEStopOn}
					className={`bg-green-500 text-white px-12 py-2 rounded w-full hover:bg-green-700 ${["IDLE_MODE", "RECOVERY_MODE"].includes(state) ? "block" : "hidden"} ${isEStopOn ? "opacity-50 cursor-not-allowed" : ""}`}
				>
					▶ Start / Resume
				</button>
				<button id="eStopBtn" onClick={toggleEStop} className="bg-red-500 text-white px-12 py-2 rounded w-full mt-2 hover:bg-red-700">
					⛔ Emergency Stop
				</button>
			</div>
		</div>
	)
}

export default JoystickControl