/////////////////////////////////////////////////////////////////////////////
// This code sample shows how you can use LibRealSense and OpenCV to display
// both an RGB stream as well as Depth stream into two separate OpenCV
// created windows.
//
/////////////////////////////////////////////////////////////////////////////

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ostream>
#include <fstream>
#include <cstring>
#include <cstddef>
#include <string>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include <iomanip>
#include <stdio.h>
#include <std_msgs/String.h>
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
using namespace std;
using namespace rs;



ros::Publisher camera_det;

// Window size and frame rate
int const INPUT_WIDTH = 320;
int const INPUT_HEIGHT = 240;
int const FRAMERATE = 60;

// Named windows
char* const WINDOW_DEPTH = "Depth Image";
char* const WINDOW_RGB = "RGB Image";


context _rs_ctx;
device* _rs_camera = NULL;
intrinsics _depth_intrin;
intrinsics           _color_intrin;
bool _loop = true;



// Initialize the application state. Upon success will return the static app_state vars address
bool initialize_streaming( )
{
bool success = false;
if( _rs_ctx.get_device_count( ) > 0 )
{
_rs_camera = _rs_ctx.get_device( 0 );

_rs_camera->enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
_rs_camera->enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );

_rs_camera->start( );

success = true;
}
return success;
}



/////////////////////////////////////////////////////////////////////////////
// If the left mouse button was clicked on either image, stop streaming and close windows.
/////////////////////////////////////////////////////////////////////////////
static void onMouse( int event, int x, int y, int, void* window_name )
{
if( event == cv::EVENT_LBUTTONDOWN )
{
_loop = false;
}
}



/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbacks.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows( )
{
cv::namedWindow( WINDOW_DEPTH, 0 );
cv::namedWindow( WINDOW_RGB, 0 );

cv::setMouseCallback( WINDOW_DEPTH, onMouse, WINDOW_DEPTH );
cv::setMouseCallback( WINDOW_RGB, onMouse, WINDOW_RGB );
}



/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
bool display_next_frame( )
{
// Get current frames intrinsic data.
_depth_intrin = _rs_camera->get_stream_intrinsics( rs::stream::depth );
_color_intrin = _rs_camera->get_stream_intrinsics( rs::stream::color );

// Create depth image
cv::Mat depth16( _depth_intrin.height,
_depth_intrin.width,
CV_16U,
(uchar *)_rs_camera->get_frame_data( rs::stream::depth ) );

// Get depth scale to meters from stream
float rs_get_device_depth_scale(const rs_device * device, rs_error ** error);
const float scale = _rs_camera->get_depth_scale();
const uint16_t * image = (const uint16_t *)_rs_camera->get_frame_data(rs::stream::depth);
ofstream myfile;
// float arr[_depth_intrin.height][_depth_intrin.width];
// double arr_L[1][1], arr_C[1][1], arr_R[1][1];
// int i=0; int j=0;
// float safety_dist = 0; float warning_dist = 0;
float depth_in_meters;
float L, C, R;
float a, b, d;
for(int dy=0; dy<_depth_intrin.height; ++dy)
        {
myfile.open ("/home/wp/depth_data_in_meters.txt", fstream::out |fstream::app);
for(int dx=0; dx<_depth_intrin.width; ++dx)
            {
            uint16_t depth_value = image[dy * _depth_intrin.width + dx];
            depth_in_meters = scale * depth_value;

            //Skip over pixels with a depth value of zero, which is used to indicate no data
//             if(depth_value == 0) continue;
//             cout << " " << float(depth_in_meters) << " ";

            //Storing depth data into a array
            //arr[_depth_intrin.height][_depth_intrin.width] = float(depth_in_meters);
            myfile << float (depth_in_meters) << "\t";

            //Displaying array depth data in meters
//             cout << arr[_depth_intrin.height][_depth_intrin.width] <<" ";
//             cout << " " << float(depth_in_meters) << " ";

            if (dy == 120)
            {


            //Finding the (consolidated) obstacle distance in front of 3 major views: Left, Center, Right
            if (dx > 40 && dx <= 90)
            {
            cout << "Warning!"<< "\n";
            	for (dx = 41; dx < 91; dx++)
            	{
            	a = depth_in_meters++;
            	}
            	L = a / 50;

            	cout << "The left side is: " << fixed << setprecision(3) << L << "\t";
            } else if (dx > 130 && dx <= 180)
            {
            	for (dx = 131; dx < 181; dx++)
            	{
            	b = depth_in_meters++;
            	}
            	C = b / 50;

            	cout << "The center is: " << C << "\t";

            } else if (dx > 230 && dx <= 280)
            {
            	for (dx = 231; dx < 281; dx++)
            	{
            	d = depth_in_meters++;
            	}
            	R = d / 50;

            	cout << "The right side is: " << R << "\n";
            }
            }
            }
            myfile << "end of row" << endl;
            //cout << "end of row" << "\n";


//				Set up publishing topic
	            std_msgs::Float64MultiArray Obsdistance; //Declare Obsdistance
	            Obsdistance.data.resize(2); //Resize the array to assign to existent values
	            Obsdistance.data[0] = R; //R to first element in the array
	            Obsdistance.data[1] = L; //L to first element in the array
	            Obsdistance.data[2] = C; //C to first element in the array
	            camera_det.publish(Obsdistance);

	            ros::spinOnce();

	}


//             else if (float L = R = C && C < 1.002)
//             {
//             cout << "Stop! ";
//             break;
//             }
        }
myfile << "end of frame" << "\n";
myfile.close();
cout << "end of frame" << "\n \n \n";
cout<<endl;


// Create color image
cv::Mat rgb( _color_intrin.height,
_color_intrin.width,
CV_8UC3,
(uchar *)_rs_camera->get_frame_data( rs::stream::color ) );

// < 800
cv::Mat depth8u = depth16;
depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );
// file  << "data" << depth8u;
// file.release();
imshow( WINDOW_DEPTH, depth8u );
cvWaitKey( 1 );

cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
imshow( WINDOW_RGB, rgb );
cvWaitKey( 1 );

return true;
}


/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( int argc, char **argv ) try
{
	//Initialize node and node name
	ros::init(argc, argv, "detection"); //Node called 'detection'
	ros::NodeHandle nh;
	//advertise as detection
	ros::Publisher camera_det = nh.advertise<std_msgs::Float64MultiArray>("/detection", 60); //Pub topic called 'Obsdistance'
	ros::Rate loop_rate(60);

rs::log_to_console( rs::log_severity::warn );

if( !initialize_streaming( ) )
{
std::cout << "Unable to locate a camera" << std::endl;
rs::log_to_console( rs::log_severity::fatal );
return EXIT_FAILURE;
}

setup_windows( );

// Loop until someone left clicks on either of the images in either window.
while( _loop )
{
if( _rs_camera->is_streaming( ) )
_rs_camera->wait_for_frames( );

display_next_frame( );
}

_rs_camera->stop( );
cv::destroyAllWindows( );

return EXIT_SUCCESS;
}
catch( const rs::error & e )
{
std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
return EXIT_FAILURE;
}
catch( const std::exception & e )
{
std::cerr << e.what() << std::endl;
return EXIT_FAILURE;
}



