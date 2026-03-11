#include "loops/line_loop.hpp"

void nodes::LineLoop::line_loop_timer_callback() {
    //if (nodes::LineNode::get_discrete_line_pose() == DiscreteLinePose::LineBoth) {
    nodes::wheel_speeds_ = {.l = 177,.r=177};

    }

