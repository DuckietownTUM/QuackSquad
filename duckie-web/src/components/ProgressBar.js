import React from "react";

const ProgressBar = ({value, max = 100, image}) => {
	const percentage = Math.min(100, (value / max) * 100);

	return (
		<div className="flex items-center relative py-2">
			{image && (
				<div className={`z-10 absolute left-[calc(100%*(${percentage}/100))] transform -translate-x-1/2 transition-all duration-300 size-7 bg-white rounded-full flex items-center justify-center`}>
					<img src={image} alt="Duckietown Logo" className="w-6 h-auto" />
				</div>
			)}
			<div className="z-1 w-full bg-gray-200 rounded-full h-4">
				<div
					className={`h-full bg-yellow-400 rounded-full transition-all duration-300 w-[${percentage}%]`}
				></div>
			</div>
		</div>
	);
};

export default ProgressBar;
