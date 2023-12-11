#include <potbot_lib/PathPlanner.h>

namespace potbot_lib{

    namespace PathPlanner{
        void create_on_APF(APF &apf, std::vector<std::vector<double>> &path, double init_robot_pose, double max_path_length, size_t path_search_range)
        {
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre;
            double map_res      = apf.potential_.resolution;
            double path_length  = 0;
            size_t range        = path_search_range;

            for (auto value : apf.potential_.values)
            {
                if (value.info[Potential::GridInfo::IS_ROBOT])
                {
                    pf_idx_min  = value.index;
                    center_x    = value.x;
                    center_y    = value.y;
                    J_min_pre   = value.value;
                    center_row  = value.row;
                    center_col  = value.col;
                    break;
                }
            }
            path.push_back({center_x, center_y});
            apf.potential_.values[pf_idx_min].info[Potential::GridInfo::IS_PLANNED_PATH] = true;

            while (apf.potential_.values[pf_idx_min].info[Potential::GridInfo::IS_AROUND_GOAL] == false && path_length <= max_path_length)
            {
                double J_min                    = J_min_pre;
                bool breakflag                  = false;
                bool solve_local_minimum        = false;
                bool found_min_potential_point  = false;
                
                std::vector<size_t> checked;
                while (found_min_potential_point  == false)
                {
                    for (size_t col = center_col-range; col <= center_col+range; col++)
                    {
                        for (size_t row = center_row-range; row <= center_row+range; row++)
                        {
                            if (row == center_row && col == center_col) continue;

                            int pf_idx = apf.potential_.get_field_index(col,row);

                            if (apf.potential_.values[pf_idx].info[Potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                            if (solve_local_minimum  == false)
                            {
                                double PotentialValue   = apf.potential_.values[pf_idx].value;
                                double x                = apf.potential_.values[pf_idx].x;
                                double y                = apf.potential_.values[pf_idx].y;

                                static double theta_pre = init_robot_pose;
                                static double x_pre     = path.back()[0];
                                static double y_pre     = path.back()[1];
                                
                                double posediff         = abs(atan2(y-y_pre,x-x_pre) - theta_pre);

                                double wu               = 1;
                                double w_theta          = 0;
                                double J                = wu*PotentialValue + w_theta*posediff;

                                if (J < J_min) 
                                {
                                    found_min_potential_point   = true;
                                    x_pre                       = x;
                                    y_pre                       = y;
                                    J_min                       = J;
                                    pf_idx_min                  = pf_idx;
                                }
                            }
                            else if (solve_local_minimum  == true)
                            {
                                //現在のJ_minを下回るまで斥力場のエッジを進んでいく処理に変更する

                                bool is_edge        = apf.potential_.values[pf_idx].info[Potential::GridInfo::IS_REPULSION_FIELD_EDGE];
                                bool is_planned     = apf.potential_.values[pf_idx].info[Potential::GridInfo::IS_PLANNED_PATH];
                                bool is_dupe        = std::find(checked.begin(), checked.end(), pf_idx) != checked.end();
                                if ((is_edge && !is_planned) || is_dupe)
                                {
                                    pf_idx_min                  = pf_idx;
                                    found_min_potential_point   = true;
                                    breakflag                   = true;
                                    break;
                                }
                                checked.push_back(pf_idx);
                            }
                        }
                        if (breakflag) break;
                    }
                    if (found_min_potential_point == false) solve_local_minimum = true;
                }
                
                double px   = apf.potential_.values[pf_idx_min].x;
                double py   = apf.potential_.values[pf_idx_min].y;
                path_length += sqrt(pow(px - path.back()[0],2) + pow(py - path.back()[1],2));
                center_row  = apf.potential_.values[pf_idx_min].row;
                center_col  = apf.potential_.values[pf_idx_min].col;
                J_min_pre   = apf.potential_.values[pf_idx_min].value;

                path.push_back({px, py});
                apf.potential_.values[pf_idx_min].info[Potential::GridInfo::IS_PLANNED_PATH] = true;

                // if (index > -1)
                // {
                //     if (breakflag || local_minimum) break;
                //     geometry_msgs::PoseStamped &pre = robot_path.poses.back();
                //     robot_pose.pose.orientation = potbot_lib::utility::get_Quat(0,0,atan2(robot_pose.pose.position.y-pre.pose.position.y,robot_pose.pose.position.x-pre.pose.position.x));
                // }
                // else
                // {
                //     robot_pose.pose.orientation = odom_.pose.pose.orientation;
                // }
                // robot_path.poses.push_back(robot_pose);

                // int idxtmp = __get_PotentialFiledIndex(robot_pose.pose.position.x,robot_pose.pose.position.y);
                // if (idxtmp != -1) potential_field_info_[idxtmp][IS_PLANNED_PATH] = true;
                // index++;
                // if (index > max_path_index_) break;
                // J_min_pre = J_min;
                // center_x = robot_pose.pose.position.x;
                // center_y = robot_pose.pose.position.y;
            }
        }
    }
}