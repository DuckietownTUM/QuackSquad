import React, { useEffect, useState } from "react"
import botImg from "../img/duckie.png"
import mapImg from "../img/map.png"

let TILES_X = 6
let TILES_Y = 7
let TILE_SIZE_M = 0.61

const Map = ({pos}) => {
	const [posX, setPosX] = useState(0)
	const [posY, setPosY] = useState(0)
	const [botWidth, setBotWidth] = useState(0)
	const [mapWidth, setMapWidth] = useState(0)
	const [mapHeight, setMapHeight] = useState(0)

	useEffect(() => {
		function updatePosition(x, y) {
			let tile_size_px = mapWidth / TILES_X
			let half_bot_width = botWidth / 2
			
			setPosX((x / TILE_SIZE_M * tile_size_px) - half_bot_width)
			setPosY((y / TILE_SIZE_M * tile_size_px) - half_bot_width)
		}
		
		setBotWidth(document.getElementById("botImg").width)
		setMapWidth(document.getElementById("mapImg").width)
		setMapHeight(document.getElementById("mapImg").height)
		updatePosition(pos[0], pos[1])
		
	}, [pos, botWidth, mapWidth])

	return (
		<div className="flex flex-col items-center min-w-6 bg-white shadow-lg rounded-lg p-8 pt-6 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-4">🗺️ Position Tracker</h1>
			<div className="relative bg-gray-200 rounded-lg p-2">
				<div className={`absolute -left-[calc((${posX / mapWidth})*50%)] -top-[calc((${posY / mapHeight})*50%)] bg-white rounded-full w-[7%] h-[6%] flex items-center justify-center`}>
					<img id="botImg" src={botImg} alt="Duckiebot" className={`w-[99%] h-auto`}/>
				</div>
				<img id="mapImg" src={mapImg} alt="Duckietown City" className=""/>
			</div>
		</div>
	)
}

export default Map