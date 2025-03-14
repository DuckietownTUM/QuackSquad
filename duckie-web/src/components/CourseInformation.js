import React from "react";
import ProgressBar from "./ProgressBar";

const CourseInformation = () => {
	return (
		<div className="flex flex-col items-center bg-white shadow-lg rounded-lg p-8 mb-2 w-full max-w-md text-center block">
			<h1 className="text-2xl font-bold mb-4">🚗 Course Information</h1>
			<div className="flex gap-2 mt-4">
				<ProgressBar value={42}/>
			</div>
		</div>
	)
};

export default CourseInformation