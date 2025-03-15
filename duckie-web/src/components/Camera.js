import React, { useState } from "react"
import ROSLIB from "roslib"
let topics = ["None", "Default [/duckie/camera_node/image/compressed]"]

const Camera = ({ros}) => {
	const [selectedTopic, setSelectedTopic] = useState(topics[0] || "")
	const [sub, setSub] = useState(null)

	function handleCallback(msg) {
		let imgElement = document.getElementById("cameraFeed");
		imgElement.src = "data:image/jpeg;base64," + msg.data;
	}

	function handleTopicSelection(topicName) {
		if (!ros)
			return

		setSelectedTopic(topicName)

		if (sub !== null)
			sub.unsubscribe()

		if (topicName === "None") {
			setSub(null)
			return
		}
		
		let newSub = new ROSLIB.Topic({
			ros: ros,
			name: topicName.split(" ")[1].substring(1, topicName.split(" ")[1].length - 1),
			messageType: "sensor_msgs/CompressedImage"
		})
		
		newSub.subscribe((msg) => handleCallback(msg))
		setSub(newSub)
	}

	return (
		<div className="flex flex-col items-center min-w-lg bg-white shadow-lg rounded-lg p-8 pt-6 mb-2 w-full text-center">
			<h1 className="text-2xl font-bold mb-4">📸 Camera Feed</h1>
			<select
				className="w-full p-2 border rounded mb-4"
				value={selectedTopic}
				onChange={(e) => handleTopicSelection(e.target.value)}
			>
				{topics.map((topic) => (
					<option key={topic} value={topic}>{topic}</option>
				))}
			</select>
			<div className="w-full h-64 bg-black flex items-center justify-center rounded">
				{sub ? (
					<img id="cameraFeed" alt="Camera Feed" className="w-full h-full object-contain" />
				) : (
					<span className="text-white">No Image Available :/</span>
				)}
			</div>
		</div>
	)
}

export default Camera