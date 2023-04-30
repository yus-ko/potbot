#include<potbot/Filter.h>

void FilterClass::mainloop()
{
    ros::Rate loop_rate(50);
    KalmanFilter obs;
    states_.push_back(obs);
	while (ros::ok())
	{
        manage();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void FilterClass::manage()
{
    filter();
}

void FilterClass::filter()
{
    //states_[0].update();
    //std::cout<<states_[0].get_state()<<std::endl;
}