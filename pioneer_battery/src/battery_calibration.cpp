#include <calibration.h>

using namespace std;

/*************GLOBAL VARIABLES***************************/

fstream battery("/home/student/ros_hydro_external/src/aau_multi_robot/pioneer_battery/param/pioneer_battery.yaml",ios::out|ios::in);

double rate = 1;// Frequency of loop [Hz]	/**********!!!!!problems by rate > 4, stay 3 and below!!!*********/		
int x=0, e=0; 
const int aw = 30;
double i= 1/rate;				// i = time [s]	
double array[2][aw];
bool ti = true;		
double ms, ms1 = 111;	
bool yh = true;

void batteryVoltageWrite(const std_msgs::Float64::ConstPtr& msg);
void batteryVoltageWrite2(const std_msgs::Float64::ConstPtr& msg);	

int calibration(int argc, char** argv)
{
	ros::init(argc, argv, "battery_calibration");
	ros::NodeHandle nh;	
	ros::spinOnce();
	ros::Rate loop_rate(rate);

	ros::Subscriber batteryVoltage_Write = nh.subscribe("/RosAria/battery_voltage", 1, &batteryVoltageWrite2);  

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();				// sleep for 1/rate seconds
	}

	battery<<"}";
	battery.close();
	return 0;
}

void batteryVoltageWrite2(const std_msgs::Float64::ConstPtr& msg) 	//subscribed topic
{
	
		
	ms = ("%F", msg->data);			//get the value out of the ROS message
	

	if(ms < ms1)
	{
		std::cout.precision(2);		// <- show more digits
		cout << 	std::fixed;																						
		cout << "Time [s]: " << i << " \t Voltage [V]:" << ("%0.2F", msg->data)<<"\t\t\t"<<e<<endl;				
		
		// write to the .yaml file

		if(yh==true)
		{
			battery<<"dict: {";
			yh=false;
		}
		else
			battery<<", ";		// battery is the name of the YAML file
	
			battery<<"\""<<(ms*10)<<"\": "<<i;
	
			e++;
			ms1 = ms;

	}
	i+=(1/rate);		//rate = lopprate in [Hz]
}

