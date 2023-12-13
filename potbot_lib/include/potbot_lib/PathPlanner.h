#ifndef _H_PATHPLANNER_
#define _H_PATHPLANNER_

#include <ros/ros.h>
#include <potbot_lib/PotentialField.h>

namespace potbot_lib{

    namespace PathPlanner{

        class APFPathPlanner : public APF{
            private:
                void __get_search_index(std::vector<size_t>& search_indexes, Field& field, size_t centor_row = 0, size_t centor_col = 0, size_t range = 1);
                double __nCr(double n, double r);
            public:
                APFPathPlanner(size_t rows = 3, size_t cols = 3, double resolution = 1.0,
                double weight_attraction_field              = 0.1,
                double weight_repulsion_field               = 0.1,
                double distance_threshold_repulsion_field   = 0.3);
                ~APFPathPlanner();
                
                void create_path(std::vector<std::vector<double>> &path, double init_robot_pose = 0.0, double max_path_length = 6.0, size_t path_search_range = 1);
                void bezier(const std::vector<std::vector<double>> &path_control, std::vector<std::vector<double>> &path_interpolated);
        };

    }
}

#endif	// _H_PATHPLANNER_