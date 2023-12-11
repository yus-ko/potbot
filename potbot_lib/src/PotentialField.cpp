#include <potbot_lib/PotentialField.h>

namespace potbot_lib{
    namespace Potential{

        Field::Field(double width, double height, double resolution)
        {
            init_field(width, height, resolution);
        }
        Field::~Field(){}

        void Field::init_field(double width, double height, double resolution)
        {
            this->width        = width;
            this->height       = height;
            this->resolution   = resolution;

            this->rows = ceil(height/resolution);
            this->cols = ceil(width/resolution);
            
            size_t field_index  = 0;

            this->x_min = -width/2.0;
            this->x_max =  width/2.0;
            this->y_min = -height/2.0;
            this->y_max =  height/2.0;

            for (size_t col = 0; col < cols; col++)
            {
                double y = this->y_max - double(col*this->resolution);
                for (size_t row = 0; row < rows; row++)
                {
                    double x = this->x_min + double(row*this->resolution);

                    Potential::FieldGrid grid;
                    grid.index  = field_index;
                    grid.x      = x;
                    grid.y      = y;
                    grid.row    = row;
                    grid.col    = col;
                    this->values.push_back(grid);
                    field_index++;
                }
            }
            // for (double y = this->y_max; y >= this->y_min; y -= this->resolution)
            // {
            //     for (double x = this->x_min; x <= this->x_max; x += this->resolution)
            //     {
            //         Potential::FieldGrid grid;
            //         grid.index  = field_index;
            //         grid.x      = x;
            //         grid.y      = y;
            //         this->values.push_back(grid);
            //         field_index++;
            //     }
            // }
        }

        void Field::search_field_info(std::vector<size_t>& result, const std::vector<size_t> terms, const std::string mode)
        {
            if (terms.empty()) return;

            if (terms.size() == 1)
            {
                for (auto value : this->values)
                {
                    if (value.info[terms[0]]) result.push_back(value.index);
                }
            }
            else if (mode == "or")
            {
                for (auto value : this->values)
                {
                    for (auto term : terms)
                    {
                        if (value.info[term])
                        {
                            result.push_back(value.index);
                            break;
                        }
                    }
                    
                }
            }
            else if (mode == "and")
            {
                for (auto value : this->values)
                {
                    bool logic_and = true;
                    for (auto term : terms)
                    {
                        logic_and *= value.info[term];
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

        size_t Field::get_field_index(double x, double y)
        {
            if(x > this->x_max || x < this->x_min)
            {
                throw std::out_of_range("invalid coordinate x argument");
            }
            else if (y > this->y_max || y < this->y_min)
            {
                throw std::out_of_range("invalid coordinate y argument");
            }

            // size_t xnum = this->width/this->resolution;
            // // size_t ynum = field_.height/field_.resolution;
            // size_t idx = abs((y-this->y_max)/this->resolution) * (xnum+1) + abs((x-this->x_min)/this->resolution);
            // return idx;

            size_t col = abs((x-this->x_min)/this->resolution);
            size_t row = abs((y-this->y_max)/this->resolution);
            size_t idx = get_field_index(row, col);
            return idx;
        }

        size_t Field::get_field_index(size_t row, size_t col)
        {
            if (row >= this->rows)
            {

                throw std::out_of_range("invalid row argument");
            }
            else if (col >= this->cols)
            {
                throw std::out_of_range("invalid col argument");
            }
            size_t idx = row*cols + col;
            return idx;
        }

        std::vector<double> Field::get_field_coordinate(size_t index)
        {
            if(index < 0 || index >= this->values.size()) throw std::out_of_range("invalid index argument");
            std::vector<double> coord(2);
            coord[0] = this->values[index].x;
            coord[1] = this->values[index].y;
            return coord;
        }

        void Field::set_field_info(size_t index, size_t meta, bool value)
        {
            if(index < this->values.size()) this->values[index].info[meta] = value;
        }

        void Field::to_pcl2(sensor_msgs::PointCloud2& pcl_msg)
        {
            // std::vector<pcl::PointXYZ> を作成
            std::vector<pcl::PointXYZ> pointVector;
            for (auto value : this->values)
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
            std::vector<size_t> filterd_index;
            search_field_info(filterd_index, terms, mode);
            for (auto idx : filterd_index)
            {
                field.values.push_back(values[idx]);
            }
        }

        void Field::info_filter(Field& field, const size_t terms)
        {
            info_filter(field, {terms}, "or");
        }
    }
}

namespace potbot_lib{

    APF::APF(double width, double height, double resolution, double weight_attraction_field, double weight_repulsion_field, double distance_threshold_repulsion_field)
    {
        weight_attraction_field_                = weight_attraction_field;
        weight_repulsion_field_                 = weight_repulsion_field;
        distance_threshold_repulsion_field_     = distance_threshold_repulsion_field;
        init_all_fields(width, height, resolution);
    }
    APF::~APF(){}

    void APF::init_attraction_field(double width, double height, double resolution)
    {
        attraction_.init_field(width, height, resolution);
    }

    void APF::init_repulsion_field(double width, double height, double resolution)
    {
        repulsion_.init_field(width, height, resolution);
    }

    void APF::init_potential_field(double width, double height, double resolution)
    {
        potential_.init_field(width, height, resolution);
    }

    void APF::init_all_fields(double width, double height, double resolution)
    {
        init_attraction_field(width, height, resolution);
        init_repulsion_field(width, height, resolution);
        init_potential_field(width, height, resolution);
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
        for(auto& value : attraction_.values)
        {
            double x = value.x;
            double y = value.y;
            double distance_to_goal = sqrt(pow(x - goal_[0],2)+pow(y - goal_[1],2));
            double attraction_value = 0.5 * weight_attraction_field * pow(distance_to_goal, 2);
            value.value = attraction_value;
            value.info[Potential::GridInfo::IS_AROUND_GOAL] = bool(distance_to_goal < 0.3);
        }
    }

    void APF::create_repulsion_field()
    {
        double distance_threshold_repulsion_field   = distance_threshold_repulsion_field_;
        double weight_repulsion_field               = weight_repulsion_field_;
        for(auto& value : repulsion_.values)
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
                    value.info[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE] = true;
                    repulsion_value += 0.5 * weight_repulsion_field * pow(1.0/(distance_to_obstacle + 1e-100) - 1.0/(distance_threshold_repulsion_field + 1e-100), 2);
                }
                else
                {
                    repulsion_value += 0;
                }
            }
            value.value = repulsion_value;
        }

        for(auto& value : repulsion_.values)
        {
            if (value.info[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
            {
                size_t rowc   = value.row;
                size_t colc   = value.col;
                bool brakeflag = false;
                for(size_t col = colc - 1; col <= colc + 1; col++)
                {
                    for(size_t row = rowc - 1; row <= rowc + 1; row++)
                    {
                        size_t next = repulsion_.get_field_index(col,row);
                        if (!repulsion_.values[next].info[Potential::GridInfo::IS_REPULSION_FIELD_INSIDE])
                        {
                            value.info[Potential::GridInfo::IS_REPULSION_FIELD_EDGE] = true;
                            brakeflag = true;
                        }
                    }
                    if (brakeflag) break;
                }
            }
        }
    }

    void APF::create_potential_field()
    {
        for(auto& value : potential_.values)
        {
            size_t idx = value.index;
            double Ua = attraction_.values[idx].value;
            double Uo = repulsion_.values[idx].value;
            double Utotal = Ua + Uo;
            value.value = Utotal;
            size_t i = 0;
            for(auto info : value.info)
            {
                info = (info || attraction_.values[idx].info[i] || repulsion_.values[idx].info[i]);
                i++;
            }
            
        }
    }

}