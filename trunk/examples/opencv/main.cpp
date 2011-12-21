
// ratslam headers
#include <Ratslam.hpp>
#include <RatslamGraphics.h>

// boost property trees
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

// opencv for video input.
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

double time_diff_to_double(unsigned long sec2, unsigned long nsec2, unsigned long sec1, unsigned long nsec1)
{
	double rsec, rnsec;

	rsec = sec2 - sec1;

	if (nsec2 > nsec1)
	{
		rnsec = nsec2 - nsec1;
	}
	else
	{
		rnsec = 1000000000 - nsec1 + nsec2;
		rsec--;
	}

	return (double)rsec + 1.0/1000000000.0*(double)rnsec;
}

int main(int argc, char * argv[])
{
	if (argc < 2)
	{
		std::cout << argv[0] << "(ERROR): Usage: ratslam config_file.txt" << std::endl;
		cin.get();
		return -1;
	}
	
	// read settings from the config file
	boost::property_tree::ptree settings;
	read_ini(argv[1], settings);
	
	// video reader for receiving camera input
	cv::VideoCapture video_reader;
	std::string dataset_video_filename;
	std::string dataset_csv_filename;
	std::string csv_line;
	std::istringstream iss;
	
	// read the video filename from the settings
	gri::get_setting_from_ptree<std::string>(dataset_video_filename, settings, "gri.robot_name", "");
	dataset_video_filename = "log_" + dataset_video_filename + ".avi";

	// read the csv filename from the settings
	gri::get_setting_from_ptree<std::string>(dataset_csv_filename, settings, "gri.robot_name", "");	
	dataset_csv_filename = "log_" + dataset_csv_filename + ".txt";		

	video_reader.open(dataset_video_filename);
	if (!video_reader.isOpened())
	{
		std::cout << argv[0] << "(ERROR): Failed to open video " << 
			dataset_video_filename << std::endl;
		cin.get();
		return -1;
	}
	
	// open the robot's log as a csv to get translational and rotational 
	// velocities, and time differences
	std::ifstream csv_in(dataset_csv_filename.c_str());
	if (!csv_in.is_open())
	{
		std::cout << argv[0] << "(ERROR): Failed to open csv file " << 
			dataset_csv_filename << std::endl;
		cin.get();
		return -1;
	}
	
	// discard the titles line
	getline(csv_in, csv_line);
	
	// fetch the ratslam settings and construct a new ratslam object
	boost::property_tree::ptree ratslam_settings;
	gri::get_setting_child(ratslam_settings, settings, "ratslam");
	ratslam::Ratslam * ratslam = new ratslam::Ratslam(ratslam_settings);
	
	// ratslam inputs
	cv::Mat frame;
	double delta_time_s, trans_vel, rot_vel;
	double time_s, last_time_s;
	unsigned long current_sec, current_nsec, last_sec, last_nsec;
	
	// construct a new ratslam graphics object to allow drawing ratslam
	boost::property_tree::ptree ratslam_graphics_settings;
	gri::get_setting_child(ratslam_graphics_settings, settings, "draw");
	IrrEventReceiver Event_Receiver;
	ratslam::RatslamGraphics * ratslam_graphics =
		new ratslam::RatslamGraphics(ratslam_graphics_settings, 
		&Event_Receiver, ratslam);


	//discard one frame
	video_reader.grab();

	// get the associated csv information
	getline(csv_in, csv_line, ',');
	iss.clear();
	iss.str(csv_line);
	iss >> last_sec;
	getline(csv_in, csv_line, ',');
	iss.clear();
	iss.str(csv_line);
	iss >> last_nsec;
		
	getline(csv_in, csv_line, '\n');

	// start a processing loop
	while (video_reader.grab() && ratslam_graphics->begin())
	{
		// get a frame from the video file 
		video_reader.retrieve(frame, 0);
		
		// get the associated csv information
		getline(csv_in, csv_line, ',');
		iss.clear();
		iss.str(csv_line);
		iss >> current_sec;
		getline(csv_in, csv_line, ',');
		iss.clear();
		iss.str(csv_line);
		iss >> current_nsec;

		delta_time_s = time_diff_to_double(current_sec, current_nsec, last_sec, last_nsec);
		last_sec = current_sec;
		last_nsec = current_nsec;

		getline(csv_in, csv_line, ',');
		iss.clear();
		iss.str(csv_line);
		iss >> trans_vel;
		getline(csv_in, csv_line, ',');
		iss.clear();
		iss.str(csv_line);
		iss >> rot_vel;
		
		getline(csv_in, csv_line, '\n');
		
		// set the ratslam inputs
		ratslam->set_view_rgb(frame.data);
		ratslam->set_odom(trans_vel, rot_vel);
		ratslam->set_delta_time(delta_time_s);
		
		// process a single frame
		ratslam->process();
		
		ratslam_graphics->end();
		
	}
	
	delete ratslam_graphics;
	delete ratslam;
	
	return 0;
}
