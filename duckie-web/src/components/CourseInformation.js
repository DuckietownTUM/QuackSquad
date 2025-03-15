import React from "react";
import ProgressBar from "./ProgressBar";
import duckIcon from "../img/duckie.png"

const CourseInformation = () => {
	return (
		<div className="bg-white shadow-lg rounded-lg p-8 mb-2 w-full max-w-md text-center">
			<h1 className="text-2xl font-bold mb-4">🚗 Course Information</h1>
			<div className="mt-4">
				<ProgressBar value={42} image={duckIcon}/>
			</div>
		</div>
	)
};

export default CourseInformation