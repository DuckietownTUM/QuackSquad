import ROSLIB from "roslib";
import React, { useEffect, useState } from "react";
import ProgressBar from "./ProgressBar";
import duckIcon from "../img/duckie.png"
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faArrowTurnDown, faArrowTurnUp, faArrowRightLong } from "@fortawesome/free-solid-svg-icons";

const CourseInformation = ({ros, pos, path}) => {
	const [nextTurn, setNextTurn] = useState(0)
	const [dist, setDist] = useState(0)
	const [speed, setSpeed] = useState(0)
	const [currentTile, setCurrentTile] = useState(0)
	const [maxTile, setMaxTile] = useState(null)

	useEffect(() => {
		if (!ros)
			return

		let nextTurnTopic = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/dijkstra_turns_node/turn_type",
			messageType: "std_msgs/Int16"
		})

		let totalDistTopic = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/deadreckoning_node/total_dist",
			messageType: "std_msgs/Float32"
		})

		let speedTopic = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/imu_node/data",
			messageType: "sensor_msgs/Imu"
		})

		let tileProgressTopic = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/dijkstra_turns_node/tile_progress",
			messageType: "dijkstra/TileProgress"
		})

		nextTurnTopic.subscribe((msg) => setNextTurn(msg.data))
		totalDistTopic.subscribe((msg) => setDist(msg.data))
		// speedTopic.subscribe((msg) => {setSpeed(msg.data)})
		tileProgressTopic.subscribe((msg) => {
			setCurrentTile(msg.current_tile)
			setMaxTile(msg.total_tile)
		})

	}, [ros])
	
	let getNextTurnArrow = () => {
		let arrow
		switch (nextTurn) {
			case 0:
				arrow = faArrowTurnUp
				break;
			case 1:
				arrow = faArrowRightLong
				break;
			case 2:
				arrow = faArrowTurnDown
				break;
			default:
				break;
		}
		return arrow
	}

	return (
		<div className="h-fit bg-white shadow-lg rounded-lg p-8 pt-6 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-4">🚗 Course Information</h1>
			<div className="grid grid-rows-[1.5fr_1fr_1fr_1fr] grid-cols-[2fr_0.25fr_3fr] items-center mt-4">
				<div className="col-span-3 mb-2">
					<ProgressBar value={currentTile} max={maxTile} image={duckIcon}/>
				</div>
				{nextTurn !== -1 ? (
					<div className="row-span-2 row-start-2 col-start-1 self-start">
						<FontAwesomeIcon icon={getNextTurnArrow()} size="3x" rotation={270} flip="" />
					</div>
				): null}
				<p className="col-start-1 row-start-4">{nextTurn === -1 ? "Available Turns": "Next Intersection"}</p>
				<div className="row-span-3 row-start-2 col-start-2 border-l-2 border-gray-400 h-full"></div>
				<p className="col-start-3 row-start-2 justify-self-start">Speed: <span>{speed.toFixed(2)}m/s</span></p>
				<p className="col-start-3 row-start-3 justify-self-start">Distance traveled: <span>{dist.toFixed(2)}m</span></p>
				<p className="col-start-3 row-start-4 justify-self-start">Tiles progression: <span>{currentTile}/{maxTile}</span></p>
			</div>
		</div>
	)
};

export default CourseInformation