#include <calibration.h>
#include <measure.h>

using namespace std;

/*************GLOBAL VARIABLES***************************/

fstream battery("/home/student/ros_hydro_external/src/aau_multi_robot/pioneer_battery/param/pioneer_battery.yaml",ios::out|ios::in);


void batteryVoltageRead();
void batteryVoltageWrite(const std_msgs::Float64::ConstPtr& msg);
int calibration(int argc, char** argv);

double rate = 1;	// Frequency of loop [Hz]	/*************!!!!!problems by rate > 4, stay 3 and below!!!!!************/

const int aw = 40;
float v;
double i=0;				// i = time [s]	
double array[2][aw];
 
const bool rw = false;			// true means only write, false means only read  (-> read or write mode)
bool rtrn = true, shown = false;			// needet to read in the values only once
double stateOfCharge;
double ms;		
double time_max;


int main(int argc, char** argv)
{
ros::init(argc, argv, "battery_measure");
ros::NodeHandle nh;	
ros::spinOnce();
ros::Rate loop_rate(rate);

ros::Publisher measure_pub = nh.advertise<std_msgs::Float32>("stateOfCharge", 1);
ros::Publisher totalTime_pub = nh.advertise<std_msgs::Float32>("totalTime", 1);

ros::Subscriber batteryVoltage_Write = nh.subscribe("/RosAria/battery_voltage", 1, &batteryVoltageWrite);  

while(ros::ok())
{
		if(rw == true)
		{
			batteryVoltage_Write.shutdown();
			calibration(argc, argv);
		}
		if ((rw == false)&&(rtrn == true))
			
			batteryVoltageRead();

			loop_rate.sleep();				// sleep for 1/rate seconds
			ros::spinOnce();	
	
			std_msgs::Float32 stoch;
			stoch.data = stateOfCharge;
			measure_pub.publish(stoch);

			std_msgs::Float32 tt;
			tt.data = time_max;
			totalTime_pub.publish(tt);
		if (shown == false)
		{
			ROS_INFO("battery state is: [%0.2F], total time is: [%0.1F]", stoch.data, tt.data);
			shown = true;
		}
	
}
	battery.close();
  return 0;
}


void batteryVoltageWrite(const std_msgs::Float64::ConstPtr& msg) 
{

	ms = ("%F", msg->data);	

	std::cout.precision(1);		// <- show more digits							
	cout << std::fixed;	

	int j = 0;
	while(j < aw)
	{
		if((array[0][j]) < (ms+0.05))
		{
			stateOfCharge = ((time_max - array[1][j])*100)/time_max;
			j = aw;
		}
		else
			j++;
		}

	ros::spinOnce();
	}

void batteryVoltageRead()
{ 

ros::NodeHandle nh;	

	double voltage = 143;
	double time;
	bool d = true;

	for(int i = 0; i <= (aw-1); i++)
	{
		if(voltage < 90)
			i = aw;

			std::stringstream param_name, place, voltageS1;
	
			voltageS1 << voltage;
			place << "pioneer_battery/dict/";
			param_name << place.str();
			param_name << voltageS1.str();
	
			if(nh.hasParam(param_name.str()))
			{
				nh.getParam(param_name.str(), time);
				d = false;
			}
			else
				d = true;

		if(d == true)
			array[0][i] = (voltage+1)/10;

		else if(d == false)
			array[0][i] = voltage/10;

			array[1][i] = time;
			voltage -= 1;
		
	}
			time_max = array[1][aw-1];

	for (int j=0; j<aw; j++)
	{
		std::cout.precision(2);		// <- show more/ less digits
		cout<<std::fixed;			
		cout<<j<<"\t"<<((time_max-array[1][j])*100)/time_max<<"%"<<"\t\t"<<array[0][j]<<"\t"<<array[1][j]<<endl;
	}
		rtrn = false;

}


