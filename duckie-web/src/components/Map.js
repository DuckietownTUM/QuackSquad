import React, { useEffect, useState } from "react"
import ROSLIB from "roslib"
import botImg from "../img/duckie.png"
import mapImg from "../img/map.png"

let TILES_X = 6
let TILES_Y = 7
let TILE_SIZE_M = 0.61

const Map = ({ros}) => {
	const [posX, setPosX] = useState(0)
	const [posY, setPosY] = useState(0)
	const [botWidth, setBotWidth] = useState(0)
	const [mapWidth, setMapWidth] = useState(0)

	useEffect(() => {
		setBotWidth(document.getElementById("botImg").width)
		setMapWidth(document.getElementById("mapImg").width)
		updatePosition(2.135, 0.305)

		if (!ros)
			return

		let odomListener = new ROSLIB.Topic({
			ros: ros,
			name: "/duckie/deadreckoning_node/odom",
			messageType: "nav_msgs/Odometry"
		})

		odomListener.subscribe((msg) => {
			updatePosition(msg.pose.pose.position.x, -msg.pose.pose.position.y);
		})

		setBotWidth(document.getElementById("botImg").width)
		setMapWidth(document.getElementById("mapImg").width)
		updatePosition(2.135, 0.305)
		
		function updatePosition(x, y) {
			let tile_size_px = mapWidth / TILES_X
			let half_bot_width = botWidth / 2
			
			setPosX((x / TILE_SIZE_M * tile_size_px) - half_bot_width)
			setPosY((y / TILE_SIZE_M * tile_size_px) - half_bot_width)
		}
		
	}, [ros, botWidth, mapWidth])

	return (
		<div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-4">🗺️ Position Tracker</h1>
			<div className="bg-gray-200 rounded-lg p-2">
				<img id="mapImg" src={mapImg} alt="Duckietown City"/>
				<img id="botImg" src={botImg} alt="Duckiebot" className={`relative size-8 left-[${posX}px] top-[${posY}px]`}/>
			</div>
		</div>
	)
}

export default Map