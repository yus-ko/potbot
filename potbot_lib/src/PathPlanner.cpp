#include <potbot_lib/PathPlanner.h>

namespace potbot_lib{

    namespace PathPlanner{
        // void get_search_index_APF(Field& field, std::vector<size_t>, )
        APFPathPlanner::APFPathPlanner(size_t rows, size_t cols, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field) : 
        APF::APF(rows, cols, resolution,weight_attraction_field, weight_repulsion_field, distance_threshold_repulsion_field){}
        APFPathPlanner::~APFPathPlanner(){}
        
        void APFPathPlanner::create_path(std::vector<std::vector<double>> &path, double init_robot_pose, double max_path_length, size_t path_search_range)
        {
            size_t center_row   = 0;
            size_t center_col   = 0;
            double center_x     = 0;
            double center_y     = 0;
            size_t pf_idx_min   = 0;
            double J_min_pre;
            double path_length  = 0;
            size_t range        = path_search_range;
            std::vector<Potential::FieldGrid>* field_values;
            Field& field = potential_;
            field_values = field.get_values();

            for (auto value : (*field_values))
            {
                if (value.states[Potential::GridInfo::IS_ROBOT])
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
            (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;

            bool solving_local_minimum = false;
            while ((*field_values)[pf_idx_min].states[Potential::GridInfo::IS_AROUND_GOAL] == false && path_length <= max_path_length)
            {
                double J_min = J_min_pre;
                
                std::vector<size_t> search_indexes;
                __get_search_index(search_indexes, field, center_row, center_col, range);
                if (search_indexes.empty()) break;

                if (solving_local_minimum == false)
                {
                    solving_local_minimum = true;
                    for (auto idx : search_indexes)
                    {
                        if ((*field_values)[idx].states[Potential::GridInfo::IS_PLANNED_PATH] == true) continue;

                        double PotentialValue   = (*field_values)[idx].value;
                        double x                = (*field_values)[idx].x;
                        double y                = (*field_values)[idx].y;

                        static double theta_pre = init_robot_pose;
                        static double x_pre     = path.back()[0];
                        static double y_pre     = path.back()[1];
                        
                        double posediff         = abs(atan2(y-y_pre,x-x_pre) - theta_pre);

                        double wu               = 1;
                        double w_theta          = 0;
                        double J                = wu*PotentialValue + w_theta*posediff;

                        if (J < J_min) 
                        {
                            solving_local_minimum       = false;
                            x_pre                       = x;
                            y_pre                       = y;
                            J_min                       = J;
                            pf_idx_min                  = idx;
                        }
                    }
                }
                else
                {
                    for (auto idx : search_indexes)
                    {
                        //現在のJ_minを下回るまで斥力場のエッジを進んでいく処理に変更する

                        bool is_edge        = (*field_values)[idx].states[Potential::GridInfo::IS_REPULSION_FIELD_EDGE];
                        bool is_planned     = (*field_values)[idx].states[Potential::GridInfo::IS_PLANNED_PATH];
                        if (is_edge && !is_planned)
                        {
                            pf_idx_min                  = idx;
                            double PotentialValue       = (*field_values)[idx].value;
                            if (PotentialValue < J_min) 
                            {
                                solving_local_minimum   = false;
                            }
                            break;
                        }
                    }
                }
                
                double px   = (*field_values)[pf_idx_min].x;
                double py   = (*field_values)[pf_idx_min].y;
                path_length += sqrt(pow(px - path.back()[0],2) + pow(py - path.back()[1],2));
                center_row  = (*field_values)[pf_idx_min].row;
                center_col  = (*field_values)[pf_idx_min].col;
                J_min_pre   = J_min;

                path.push_back({px, py});
                (*field_values)[pf_idx_min].states[Potential::GridInfo::IS_PLANNED_PATH] = true;
                
            }
        }

        void APFPathPlanner::__get_search_index(std::vector<size_t>& search_indexes, Field& field, size_t centor_row, size_t centor_col, size_t range)
        {
            for (size_t row = centor_row-range; row <= centor_row+range; row++)
            {
                for (size_t col = centor_col-range; col <= centor_col+range; col++)
                {
                    if (row == centor_row && col == centor_col) continue;
                    try 
                    {
                        int pf_idx = field.get_field_index(row,col);
                        search_indexes.push_back(pf_idx);
                    }
                    catch(std::out_of_range& oor) 
                    {
                        search_indexes.clear();
                        return;
                    }
                    
                }
            }
        }

        void APFPathPlanner::bezier(const std::vector<std::vector<double>> &path_control, std::vector<std::vector<double>> &path_interpolated)
        {
            // https://www.f.waseda.jp/moriya/PUBLIC_HTML/education/classes/infomath6/applet/fractal/spline/

            path_interpolated.clear();

            int n = path_control.size();

            int bezier_idx = 0;
            double inc = 1.0/double(n*10);
            for (double t = 0.0; t <= 1.0; t += inc)
            {
                double x = path_control[bezier_idx][0];
                double y = path_control[bezier_idx][1];
                for (double i = 0.0; i <= n-1.0; i++)
                {
                    x += __nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)][0];
                    y += __nCr(n-1.0,i) * pow(t,i) * pow(1.0-t,n-i-1.0) * path_control[size_t(i)][1];
                }
                path_interpolated.push_back({x,y});
            }
        }

        double APFPathPlanner::__nCr(double n, double r)
        {
            double top = 1.0;
            double bottom = 1.0;

            for(double i = 0.0; i < r; i++)
            {
                top *= n-i;
            }

            for(double i = 0.0; i < r; i++)
            {
                bottom *= i+1.0;
            }
            
            return top/bottom;
        }
    }
}