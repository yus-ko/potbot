#include <potbot_lib/PotentialField.h>

namespace potbot_lib{
    namespace Potential{

        Field::Field(size_t rows, size_t cols, double resolution)
        {
            init_field(rows, cols, resolution);
        }
        Field::~Field(){}

        void Field::init_field(size_t rows, size_t cols, double resolution)
        {
            values_.clear();

            header_.rows            = rows;
            header_.cols            = cols;

            header_.width           = resolution*(double)cols;
            header_.height          = resolution*(double)rows;
            header_.resolution      = resolution;
            
            size_t field_index      = 0;

            // header_.x_shift    = -header_.resolution/2.0 - header_.width/2.0;
            // header_.y_shift    = -header_.resolution/2.0 - header_.height/2.0;

            header_.x_shift         = -header_.width/2.0;
            header_.y_shift         = -header_.height/2.0;

            header_.x_min           = -header_.width/2.0 + header_.x_shift;
            header_.x_max           = header_.width/2.0 + header_.x_shift;
            header_.y_min           = -header_.height/2.0 + header_.y_shift;
            header_.y_max           = header_.height/2.0 + header_.y_shift;

            for (size_t row = 0; row < header_.rows; row++)
            {
                double y            = (double)row*header_.resolution + header_.y_shift;
                for (size_t col = 0; col < header_.cols; col++)
                {
                    double x        = (double)col*header_.resolution + header_.x_shift;
                    
                    Potential::FieldGrid grid;
                    grid.index      = field_index;
                    grid.x          = x;
                    grid.y          = y;
                    grid.row        = row;
                    grid.col        = col;
                    values_.push_back(grid);
                    field_index++;
                }
            }
            // for (double y = header_.y_max; y >= header_.y_min; y -= header_.resolution)
            // {
            //     for (double x = header_.x_min; x <= header_.x_max; x += header_.resolution)
            //     {
            //         Potential::FieldGrid grid;
            //         grid.index  = field_index;
            //         grid.x      = x;
            //         grid.y      = y;
            //         values_.push_back(grid);
            //         field_index++;
            //     }
            // }
        }

        void Field::set_values(std::vector<FieldGrid>& values)
        {
            if (values_.size() == values.size()) values_ = values;
        }

        void Field::set_value(FieldGrid value)
        {
            size_t idx = value.index;
            check_index(idx);
            values_[idx] = value;
        }

        void Field::search_field_info(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode)
        {
            if (terms.empty()) return;

            if (terms.size() == 1)
            {
                for (auto value : values_)
                {
                    if (value.states[terms[0]]) result.push_back(value.index);
                }
            }
            else if (mode == "or")
            {
                for (auto value : values_)
                {
                    for (auto term : terms)
                    {
                        if (value.states[term])
                        {
                            result.push_back(value.index);
                            break;
                        }
                    }
                    
                }
            }
            else if (mode == "and")
            {
                for (auto value : values_)
                {
                    bool logic_and = true;
                    for (auto term : terms)
                    {
                        logic_and *= value.states[term];
                        if (!logic_and) break;
                    }
                    if (logic_and) result.push_back(value.index);
                }
            }
        }

        void Field::search_field_info(std::vector<size_t>& result, const size_t term)
        {
            search_field_info(result, {term}, "or");
        }

        int Field::check_index(auto index)
        {
            if(index < 0 || index >= values_.size()) throw std::out_of_range("invalid index argument");
            return 0;
        }

        FieldHeader Field::get_header()
        {
            return header_;
        }

        std::vector<FieldGrid>* Field::get_values()
        {
            return &values_;
        }

        FieldGrid Field::get_value(size_t index)
        {
            check_index(index);
            return values_[index];
        }

        FieldGrid Field::get_value(double x, double y)
        {
            return get_value(get_field_index(x,y));
        }

        size_t Field::get_field_index(double x, double y)
        {
            if(x > header_.x_max || x < header_.x_min)
            {
                throw std::out_of_range("invalid coordinate x argument");
            }
            else if (y > header_.y_max || y < header_.y_min)
            {
                throw std::out_of_range("invalid coordinate y argument");
            }
            
            size_t col = (x - header_.x_shift)/header_.resolution;
            size_t row = (y - header_.y_shift)/header_.resolution;
            size_t idx = get_field_index(row, col);
            return idx;
        }

        size_t Field::get_field_index(size_t row, size_t col)
        {
            if (row >= header_.rows)
            {

                throw std::out_of_range("invalid row argument");
            }
            else if (col >= header_.cols)
            {
                throw std::out_of_range("invalid col argument");
            }
            size_t idx = row*header_.cols + col;
            return idx;
        }

        std::vector<double> Field::get_field_coordinate(size_t index)
        {
            check_index(index);
            std::vector<double> coord(2);
            coord[0] = values_[index].x;
            coord[1] = values_[index].y;
            return coord;
        }

        void Field::set_field_info(size_t index, size_t meta, bool value)
        {
            if(index < values_.size()) values_[index].states[meta] = value;
        }

        void Field::to_pcl2(sensor_msgs::PointCloud2& pcl_msg)
        {
            // std::vector<pcl::PointXYZ> を作成
            std::vector<pcl::PointXYZ> pointVector;
            for (auto value : values_)
            {
                double x = value.x;
                double y = value.y;
                double z = value.value;
                pcl::PointXYZ point(x,y,z);
                pointVector.push_back(point);
            }

            // std::vector<pcl::PointXYZ> を pcl::PointCloud に変換
            pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pclPointCloud->points.resize(pointVector.size());
            for (size_t i = 0; i < pointVector.size(); ++i) {
                pclPointCloud->points[i] = pointVector[i];
            }

            // pcl::PointCloud を sensor_msgs::PointCloud2 に変換
            pcl::toROSMsg(*pclPointCloud, pcl_msg);
        }

        void Field::info_filter(Field& field, const std::vector<size_t> terms, const std::string mode)
        {
            field.values_.clear();
            std::vector<size_t> filterd_index;
            search_field_info(filterd_index, terms, mode);
            for (auto idx : filterd_index)
            {
                field.values_.push_back(values_[idx]);
            }
        }

        void Field::info_filter(Field& field, const size_t terms)
        {
            info_filter(field, {terms}, "or");
        }
    }
}

namespace potbot_lib{

    APF::APF(size_t rows, size_t cols, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field) : 
    Potential::Field::Field(rows, cols, resolution)
    {
        weight_attraction_field_                = weight_attraction_field;
        weight_repulsion_field_                 = weight_repulsion_field;
        distance_threshold_repulsion_field_     = distance_threshold_repulsion_field;
        init_all_fields(rows, cols, resolution);
    }
    APF::~APF(){}

    void APF::init_attraction_field(size_t rows, size_t cols, double resolution)
    {
        attraction_.init_field(rows, cols, resolution);
    }

    void APF::init_repulsion_field(size_t rows, size_t cols, double resolution)
    {
        repulsion_.init_field(rows, cols, resolution);
    }

    void APF::init_potential_field(size_t rows, size_t cols, double resolution)
    {
        potential_.init_field(rows, cols, resolution);
    }

    void APF::init_all_fields(size_t rows, size_t cols, double resolution)
    {
        init_attraction_field(rows, cols, resolution);
        init_repulsion_field(rows, cols, resolution);
        init_potential_field(rows, cols, resolution);
    }

    void APF::set_goal(size_t index)
    {
        attraction_.set_field_info(index, Potential::GridInfo::IS_GOAL, true);
        repulsion_.set_field_info(index, Potential::GridInfo::IS_GOAL, true);
        potential_.set_field_info(index, Potential::GridInfo::IS_GOAL, true);
    }

    void APF::set_robot(size_t index)
    {
        attraction_.set_field_info(index, Potential::GridInfo::IS_ROBOT, true);
        repulsion_.set_field_info(index, Potential::GridInfo::IS_ROBOT, true);
        potential_.set_field_info(index, Potential::GridInfo::IS_ROBOT, true);
    }

    void APF::set_obstacle(size_t index)
    {
        attraction_.set_field_info(index, Potential::GridInfo::IS_OBSTACLE, true);
        repulsion_.set_field_info(index, Potential::GridInfo::IS_OBSTACLE, true);
        potential_.set_field_info(index, Potential::GridInfo::IS_OBSTACLE, true);
    }

    void APF::set_goal(double x, double y)
    {
        goal_ = {x,y};
        try 
        {
            set_goal(potential_.get_field_index(x,y));
        }
        catch(...){}
        
    }

    void APF::set_robot(double x, double y)
    {
        robot_ = {x,y};
        try 
        {
            set_robot(potential_.get_field_index(x,y));
        }
        catch(...){}
    }

    void APF::set_obstacle(double x, double y)
    {
        obstacles_.push_back({x,y});
        try 
        {
            set_obstacle(potential_.get_field_index(x,y));
        }
        catch(...){}
    }

    void APF::get_attraction_field(Potential::Field& field)
    {
        field = attraction_;
    }

    void APF::get_repulsion_field(Potential::Field& field)
    {
        field = repulsion_;
    }

    void APF::get_potential_field(Potential::Field& field)
    {
        field = potential_;
    }

    size_t APF::get_goal_index(Potential::Field& field)
    {
        std::vector<size_t> result;
        field.search_field_info(result, Potential::GridInfo::IS_GOAL);
        return result[0];
    }
    
    size_t APF::get_robot_index(Potential::Field& field)
    {
        std::vector<size_t> result;
        field.search_field_info(result, Potential::GridInfo::IS_ROBOT);
        return result[0];
    }
    
    std::vector<size_t> APF::get_obstacle_indexes(Potential::Field& field)
    {
        std::vector<size_t> result;
        field.search_field_info(result, Potential::GridInfo::IS_OBSTACLE);
        return result;
    }

    void APF::create_attraction_field()
    {
        double weight_attraction_field = weight_attraction_field_;
        std::vector<Potential::FieldGrid>* attraction_values;
        attraction_values = attraction_.get_values();
        for(auto& value : (*attraction_values))
        {
            double x = value.x;
            double y = value.y;
            double distance_to_goal = sqrt(pow(x - goal_[0],2)+pow(y - goal_[1],2));
            double attraction_value = 0.5 * weight_attraction_field * pow(distance_to_goal, 2);
            value.value = attraction_value;
            value.states[Potential::GridInfo::IS_AROUND_GOAL] = bool(distance_to_goal < 0.3);
        }
    }

    void APF::create_repulsion_field()
    {
        double distance_threshold_repulsion_field   = distance_threshold_repulsion_field_;
        double weight_repulsion_field               = weight_repulsion_field_;
        std::vector<Potential::FieldGrid>* repulsion_values;
        repulsion_values = repulsion_.get_values();
        for(auto& value : (*repulsion_values))
        {
            double x = value.x;
            double y = value.y;
            double repulsion_value = 0;
            for(auto& coord_obstacle : obstacles_)
            {
                double x_obstacle = coord_obstacle[0];
                double y_obstacle = coord_obstacle[1];
                double distance_to_obstacle = sqrt(pow(x-x_obstacle,2) + pow(y-y_obstacle,2));
                if (distance_to_obstacle <= distance_threshold_repulsion_field)
                {
                    value.states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE] = true;
                    repulsion_value += 0.5 * weight_repulsion_field * pow(1.0/(distance_to_obstacle + 1e-100) - 1.0/(distance_threshold_repulsion_field + 1e-100), 2);
                }
                else
                {
                    repulsion_value += 0;
                }
            }
            value.value = repulsion_value;
        }

        for(auto& value : (*repulsion_values))
        {
            if (value.states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
            {
                size_t rowc   = value.row;
                size_t colc   = value.col;
                bool brakeflag = false;
                for(size_t row = rowc - 1; row <= rowc + 1; row++)
                {
                    for(size_t col = colc - 1; col <= colc + 1; col++)
                    {
                        if (row == rowc && col == colc) continue;
                        try 
                        {
                            size_t next = repulsion_.get_field_index(row,col);
                            if ((*repulsion_values)[next].states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE] == false)
                            {
                                value.states[Potential::GridInfo::IS_REPULSION_FIELD_EDGE] = true;
                                brakeflag = true;
                            }
                        }
                        catch(std::out_of_range& oor) 
                        {
                            continue;
                        }
                    }
                    if (brakeflag) break;
                }
            }
        }
    }

    void APF::create_potential_field()
    {
        std::vector<Potential::FieldGrid>* potential_values;
        potential_values = potential_.get_values();
        for(auto& value : (*potential_values))
        {
            size_t idx = value.index;
            double Ua = attraction_.get_value(idx).value;
            double Uo = repulsion_.get_value(idx).value;
            double Utotal = Ua + Uo;
            value.value = Utotal;
            size_t i = 0;
            for(auto info : value.states)
            {
                info = (info || attraction_.get_value(idx).states[i] || repulsion_.get_value(idx).states[i]);
                i++;
            }
            
        }

        for(auto& value : (*potential_values))
        {
            if (value.states[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
            {
                size_t rowc   = value.row;
                size_t colc   = value.col;
                bool local_minimum = false;
                for(size_t row = rowc - 1; row <= rowc + 1; row++)
                {
                    for(size_t col = colc - 1; col <= colc + 1; col++)
                    {
                        if (row == rowc && col == colc) continue;
                        try 
                        {
                            size_t next = potential_.get_field_index(row,col);
                            if ((*potential_values)[next].value < value.value)
                            {
                                local_minimum = true;
                            }
                        }
                        catch(std::out_of_range& oor) 
                        {
                            continue;
                        }
                    }
                }

                if(!local_minimum && !value.states[Potential::GridInfo::IS_GOAL])
                {
                    value.states[Potential::GridInfo::IS_LOCAL_MINIMUM] = true;
                }
            }
        }
    }

}