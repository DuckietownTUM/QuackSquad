import React from "react";

const ProgressBar = ({value, max = 100, image}) => {
	const percentage = Math.min(100, (value / max) * 100);

	return (
		<div className="relative w-full">
			{image && (
				<img
					src={image}
					alt="Progress Icon"
					className="absolute top-[-20px] left-[calc(100%*(${percentage}/100))] transform -translate-x-1/2"
					style={{ width: "24px", height: "24px" }}
				/>
			)}
			<div className="w-full bg-gray-200 rounded-full h-4 overflow-hidden block">
				<div
					className={`h-full bg-yellow-400 transition-all duration-300`}
					style={{ width: `${percentage}%` }}
				></div>
			</div>
		</div>
	);
};

export default ProgressBar;
