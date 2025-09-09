#ifndef MARIO_HPP
#define MARIO_HPP
namespace mario {

	struct rsHandle {
	  rs2::frame_queue frame_q;
	  rs2::pointcloud pc;
	  rs2::pipeline pipe;
	  rs2::align align;
	  rs2::config cfg;
	};

} // namespace mario
