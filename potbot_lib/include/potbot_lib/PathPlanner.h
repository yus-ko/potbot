#ifndef _H_PATHPLANNER_
#define _H_PATHPLANNER_

#include <ros/ros.h>
#include <potbot_lib/PotentialField.h>

namespace potbot_lib{

    namespace PathPlanner{

        class APFPathPlanner : public APF{
            public:
                using APF::APF;
                void create_path(std::vector<std::vector<double>> &path, double init_robot_pose = 0.0, double max_path_length = 6.0, size_t path_search_range = 1);
        };

    }
}

#endif	// _H_PATHPLANNER_