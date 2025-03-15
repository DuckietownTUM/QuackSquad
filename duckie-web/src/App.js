import React, {useState, useEffect} from "react"
import ROSLIB from "roslib"
import CourseInformation from "./components/CourseInformation"
import PathFinder from "./components/PathFinder"
import JoystickControl from "./components/JoystickControl"
import Camera from "./components/Camera"
import StatusBar from "./components/StatusBar";
import Map from "./components/Map"
// import { Button } from "@/components/ui/button"

const ROSBridgeURL = "ws://duckie.local:9090" // Change this if needed

const App = () => {
	const [ros, setRos] = useState(null)
	const [isEStopOn, setisEStopOn] = useState(false)
	const [state, setState] = useState("IDLE_MODE")
	const [rosError, setRosError] = useState(null)

	useEffect(() => {
		const ros = new ROSLIB.Ros({ url: ROSBridgeURL })

		ros.on("connection", () => {
			console.log("Connected to ROS")
		
			let eStopTopic = new ROSLIB.Topic({
				ros: ros,
				name: "/duckie/wheels_driver_node/emergency_stop",
				messageType: "duckietown_msgs/BoolStamped"
			})
		
			let modeTopic = new ROSLIB.Topic({
				ros: ros,
				name: "/duckie/fsm_node/mode",
				messageType: "duckietown_msgs/FSMState"
			})
		
			// Connect to the service
		
			eStopTopic.subscribe((msg) => setisEStopOn(msg.data))
			modeTopic.subscribe((msg) => setState(msg.state))
		
			setRos(ros)
			console.log(ros)
		})
	
		ros.on("error", (error) => {
			console.error("ROS error: ", error)
			setRosError(error)
		})
	
		ros.on("close", () => {
			console.log("Disconnected from ROS")
		
			setState(null)
		})

		return () => {
			ros.close()
		}
	}, [])

	return (
    <div className="flex flex-col items-center px-6 pb-6 pt-16 bg-gray-100 min-h-screen">
		<StatusBar ros={ros} state={state} error={rosError}/>
		<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4 p-4">
			{state === 'IDLE_MODE' ? (
				<PathFinder ros={ros}/>
			):(
				<CourseInformation/>
			)}
			<JoystickControl ros={ros} state={state} isEStopOn={isEStopOn}/>
			<Camera ros={ros}/>
			<Map ros={ros}/>
		</div>
	</div>
	)
}

export default App
