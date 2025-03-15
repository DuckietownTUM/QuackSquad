import React, {useEffect, useState} from "react"
import ROSLIB from "roslib"
import CourseInformation from "./CourseInformation"
import Map from "./Map"

const Tracker = ({ros}) => {
	const [pos, setPos] = useState([2.135, 0.508])

	useEffect(() => {
		if (!ros)
			return

		let odomListener = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/deadreckoning_node/odom",
			messageType: "nav_msgs/Odometry"
		})

		odomListener.subscribe((msg) => setPos((msg.pose.pose.position.x, msg.pose.pose.position.y)))
	}, [ros])

	return (
		<span>
			<CourseInformation ros={ros} pos={[2.135, 0.508]} path={null} />
			<Map ros={ros} pos={[2.135, 0.508]} />
		</span>
	)
}

export default Tracker