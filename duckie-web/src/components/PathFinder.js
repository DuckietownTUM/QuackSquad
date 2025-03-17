import React, {useState, useEffect} from "react";
import roslib from "roslib";

const PathFinder = ({ros}) => {
	const [startPoint, setStartPoint] = useState(null);
	const [destPoint, setDestPoint] = useState(null);
	const [computePathSrv, setComputePathSrv] = useState(null)

	useEffect(() => {
		if (!ros)
			return
		
		let computePathSrv = new roslib.Service({
			ros: ros,
			name: '/duckie/dijkstra_turns_node/compute_path',
			serviceType: 'dijkstra/SetRoute', // Change the type if the service has a specific one
		});

		setComputePathSrv(computePathSrv)
	}, [ros]);

	const computePath = () => {
		// Regex to match [digit][any character][digit]
		const regex = /^\d\D\d$/;

		// Validate start and destination points
		if (!regex.test(startPoint) || !regex.test(destPoint)) {
			alert("Both start and destination points should be in the format [digit][any character][digit] (e.g., 1a1).");
			return;
		}

		let req = new roslib.ServiceRequest({
			start_point: { x: parseFloat(startPoint[0]), y: parseFloat(startPoint[2]), z: 0.0 },
			dest_point: { x: parseFloat(destPoint[0]), y: parseFloat(destPoint[2]), z: 0.0 }
		});

		computePathSrv.callService(req, (res) => {
			console.log(res)
			if (res.success) {
				console.log("Path computed successfully!");
			}
			else {
				console.log("Failed to compute path");
			}
		})
	}

	return (
		<div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 pt-6 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-4">📍 Duckiebot Path Finder</h1>
			<div className="flex flex-col gap-2 mt-4">
				<input
					type="text"
					placeholder="Enter start point"
					value={startPoint}
					onChange={(e) => setStartPoint(e.target.value)}
					className="w-full px-4 py-2 border rounded mb-2"
				/>
				<input
					type="text"
					placeholder="Enter destination point"
					value={destPoint}
					onChange={(e) => setDestPoint(e.target.value)}
					className="w-full px-4 py-2 border rounded mb-4"
				/>
				<button id="computePath" onClick={computePath} disabled={!startPoint || !destPoint} className={`px-12 py-2 rounded w-full bg-yellow-400 ${!startPoint || !destPoint ? "opacity-50 cursor-not-allowed" : "hover:bg-yellow-500 text-white"}`} >
					Continue
				</button>
			</div>
		</div>
	)
}

export default PathFinder