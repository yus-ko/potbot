#include<autonomous_mobile_robot_2022/PotentialMethod.h>

void PotentialMethodClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("ROBOT_NAME",ROBOT_NAME);
    n.getParam("PATH_PLANNING_METHOD",PATH_PLANNING_METHOD);
    n.getParam("PATH_PLANNING_FILE",PATH_PLANNING_FILE);
    n.getParam("USE_AMCL",USE_AMCL);
    n.getParam("IS_SIMULATOR",IS_SIMULATOR);
    n.getParam("PUBLISH_COMMAND",PUBLISH_COMMAND);
    n.getParam("PID_CONTROL",PID_CONTROL);
    n.getParam("ANGLE_CORRECTION",ANGLE_CORRECTION);
    n.getParam("MAX_VELOCITY",MAX_VELOCITY);
    n.getParam("MAX_ANGULAR",MAX_ANGULAR);
    n.getParam("TARGET_POSITION_X",TARGET_POSITION_X);
    n.getParam("TARGET_POSITION_Y",TARGET_POSITION_Y);
    n.getParam("GAIN_PROPORTIONAL",GAIN_PROPORTIONAL);
    n.getParam("GAIN_INTEGRAL",GAIN_INTEGRAL);
    n.getParam("GAIN_DIFFERENTIAL",GAIN_DIFFERENTIAL);
    n.getParam("AVOIDANCE_RADIUS",rho_zero);
    n.getParam("PATH_TRACKING_MARGIN",PATH_TRACKING_MARGIN);
    n.getParam("POTENTIAL_FIELD_WIDTH",POTENTIAL_FIELD_WIDTH);
    n.getParam("POTENTIAL_FIELD_DIVDE_X",POTENTIAL_FIELD_DIVDE_X);
    n.getParam("POTENTIAL_FIELD_DIVDE_Y",POTENTIAL_FIELD_DIVDE_Y);

}