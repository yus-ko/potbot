#include<potbot/Controller.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"potbot_co");

    ControllerClass cc;

	// std::string s;
    // std::cout << "Press Enter to start:";
    // std::cin >> s;

	cc.mainloop();

	return 0;
}