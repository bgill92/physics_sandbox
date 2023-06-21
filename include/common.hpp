#pragma once

#include <Eigen/Dense>

namespace common {

	struct State {
	// public:
	// 	void set_position(Eigen::Vector3f const& position) {
	// 		position_ = position;
	// 	}
	// 	void set_linear_velocity(Eigen::Vector3f const& linear_velocity) {
	// 		linear_velocity_ = linear_velocity;
	// 	}
	// 	Eigen::Vector3f get_position() const {
	// 		return position_;
	// 	}
	// 	Eigen::Vector3f get_linear_velocity() const {
	// 		return linear_velocity_;
	// 	}
	// private:
		Eigen::Vector3d position;
		Eigen::Vector3d linear_velocity;
		Eigen::Vector3d linear_acceleration;
	};

}