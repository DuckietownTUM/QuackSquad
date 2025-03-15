import React from "react";

const ProgressBar = ({ value, max = 100, image}) => {
	const percentage = Math.min(100, (value / max) * 100);

	return (
		<div className="">
			{image && (
				<div className={`z-10 relative left-[calc(100%*(${percentage}/100))] transform -translate-x-1/2 w-7 h-7 bg-white rounded-full flex items-center justify-center`}
					style={{ "--progress": percentage / 100 }}
				>
					<img src={image} alt="Duckietown Logo" className="w-6 h-6" />
				</div>
			)}
			<div className="z-1 relative top-[-22px] w-full bg-gray-200 rounded-full h-4">
				<div
					className={`h-full bg-yellow-400 rounded-full transition-all duration-300`}
					style={{ width: `${percentage}%` }}
				></div>
			</div>
		</div>
	);
};

export default ProgressBar;
