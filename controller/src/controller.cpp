//All credit goes to Kasper A. Norstrøm & Kenneth R. Larsen

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <msgs/RemoteControl.h>
#include <fstream>
#include <ctime>
using namespace std;
using namespace cv;

typedef int16_t int16;
typedef int8_t int8;

int pic_size = 0;
int img_col = 0;
int img_row = 0;
int center = 0;
static const string OPENCV_WINDOW = "Image window";
Mat frame;

struct RemoteControl{
bool remote_connected = true;
bool emergency_stop = false;
bool deadman_state = true;
int16 switches = 0;
int16 buttons = 0;
int16 analog_button_a = 0;
int16 analog_button_b = 0;
int16 analog_button_c = 0;
int16 analog_button_d = 0;
int8 digital_joy_a = 0;
int8 digital_joy_b = 0;
int8 analog_joy_a_up_down = 0;
int8 analog_joy_a_left_right = 0;
int8 analog_joy_b_up_down = 0;
int8 analog_joy_b_left_right = 0;
int8 rc_battery_percentage = 100;
bool rc_battery_low_warning = false;
bool rc_battery_low_alert = false;
};

msgs::RemoteControl rc_msg;

//Actuation enable message template
void enable_actuation_message(){
	rc_msg.remote_connected = true;
	rc_msg.emergency_stop = false;
	rc_msg.deadman_state = true;
	rc_msg.switches = 0;
	rc_msg.buttons = 0;
	rc_msg.analog_button_a = 0;
	rc_msg.analog_button_b = 0;
	rc_msg.analog_button_c = 0;
	rc_msg.analog_button_d = 0;
	rc_msg.digital_joy_a = 0;
	rc_msg.digital_joy_b = 0;
	rc_msg.analog_joy_a_up_down = 0;
	rc_msg.analog_joy_a_left_right = 0;
	rc_msg.analog_joy_b_up_down = 0;
	rc_msg.analog_joy_b_left_right = 0;
	rc_msg.rc_battery_percentage = 100;
	rc_msg.rc_battery_low_warning = false;
	rc_msg.rc_battery_low_alert = false;
}

//Steering template
void steering_message(int steering, int speed){
	rc_msg.remote_connected = true;
	rc_msg.emergency_stop = false;
	rc_msg.deadman_state = true;
	rc_msg.switches = 0;
	rc_msg.buttons = 0;
	rc_msg.analog_button_a = 0;
	rc_msg.analog_button_b = 0;
	rc_msg.analog_button_c = 0;
	rc_msg.analog_button_d = 0;
	rc_msg.digital_joy_a = 0;
	rc_msg.digital_joy_b = 0;
	rc_msg.analog_joy_a_up_down = speed;
	rc_msg.analog_joy_a_left_right = steering;
	rc_msg.analog_joy_b_up_down = 0;
	rc_msg.analog_joy_b_left_right = 0;
	rc_msg.rc_battery_percentage = 100;
	rc_msg.rc_battery_low_warning = false;
	rc_msg.rc_battery_low_alert = false;
}

double Kp = 1;
double Ki = 0.7;
double Kd = 0.7;
double i_term = 0;
double output = 0;
double err = 0;
double err_sum = 0;
double last_err = 0;
double set_point = center;
double d_err = 0;
int diller = 0;
double test[3][1000];
int s = 0;

clock_t start = clock();

void steer_output() {
    diller++;
    if(diller > 999){
        diller = 999;
    }
    test[0][diller] = s;
    test[2][diller] = (clock()-start)/((double) CLOCKS_PER_SEC);
    //rc_msg.analog_joy_a_up_down = -10;
    if(s == pic_size/2){
        rc_msg.analog_joy_a_left_right = 0;
    }
    else {
    
    //ROS_INFO_STREAM("SteerDir " << s);
    
    //Error calculation
    err = set_point - s;
    //Error for Integrator
    err_sum = err_sum + err;
    //Integrator error anti-windup
    if(err_sum < -100){
        err_sum = -100;
    }
    if(err_sum > 100){
        err_sum = 100;
    }
    //Error for Derivative
    d_err = err - last_err;

    //Anti-windup Integrator
    i_term = err_sum*Ki;
    if(i_term > 100){
        i_term = 100;
    }
    if(i_term < -100){
        i_term = -100;
    }
    //output calculation
    output = ((Kp*err)+i_term+(Kd*d_err));
    //output = Kp*err;  //For a P-controller only
    //Update error variable
    last_err = err;
    //Output saturation
    if(output > 100){
        output = 100;
        rc_msg.analog_joy_a_left_right = -100;
        test[1][diller] = 100;
    }
    if(output < -100){
        output = -100;
        rc_msg.analog_joy_a_left_right = 100;
        test[1][diller] = -100;
    }
    if(output < 100 && output > -100){
        rc_msg.analog_joy_a_left_right = (output*-1);
        test[1][diller] = (output);
    }
    }
    //ROS_INFO_STREAM("PID_output" << output);

}

void cannyImage(Mat img) {

	//Placeholders
	Mat binaryBillede, bin1, bin2, blurBillede;	

	//Binary Images	
	//threshold(img, bin1, 100,255,THRESH_BINARY);
	//threshold(img, bin2,255,255,THRESH_BINARY_INV);
	//bitwise_and(bin1,bin2,binaryBillede);

	//kun til test binary
	threshold(img,binaryBillede,85,255,THRESH_BINARY);

	//Gaussian Blur
	GaussianBlur(binaryBillede, blurBillede,Size(31,31),0,0);

	// Canny
	Mat dst, cdst;
	Canny(blurBillede, dst, 25, 150, 3);
	cvtColor(dst, cdst, CV_GRAY2RGB);	//Laver cdst om til RGB - dog BGR cause opencv sucks!!
	
	//Morphology
	Mat mof(40,40,CV_8U,Scalar(1));	//Search Mask
	Mat testMof; 
	dilate(dst,testMof,mof);

	// Houghline med canny
	vector<Vec4i> lines;
	HoughLinesP(testMof, lines, 1, CV_PI / 180, 50, 150, 1);
	
	for (size_t i = 0; i < lines.size(); i++) {
		int lenX = 0;
		int lenY = 0;

		Vec4i l = lines[i];
		lenX = abs(l[2] - l[0]);
		lenY = abs(l[3] - l[1]);
		//(x0,y0,x1,y1)
		if (lenX<lenY) {
			line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);
		}
		else {
			line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, CV_AA);
		}
	}

	// lange løg
	//variabler
	int midRows = cdst.rows/2;
	int midCols = cdst.cols/2;
    ROS_INFO_STREAM("PicSise " << midCols);

	int topendRed=0;
	int maxtopred=0;

	//forlængere red
	for( int c = midCols-80; c<midCols+80;c++)
	{
		for( int r = 0; r<midRows;r++)
		{
			if(cdst.at<Vec3b>(r,c)[2]==255)
			{
				topendRed++;
			}
		}
		if(topendRed>(midRows/4))
		{
			for(int t = 0; t<midRows;t++)
			{
				cdst.at<Vec3b>(t,c)[2]=255;
				cdst.at<Vec3b>(t,c)[0]=0;
			}
		}
		if (maxtopred<topendRed)
		{
			maxtopred=topendRed;
		}
		topendRed=0;
	} 

	// Steering direction from detected line
	center = cdst.cols / 2;
	pic_size = cdst.cols;
	//int steering=0;
	int blackCount=0;
    int redCount = 0;
	// i = rows,  j = cols

	int i = 100;
	for (int j = 0; j<cdst.cols; j += 1) {
		if (cdst.at<Vec3b>(i, j)[2] == 255 && cdst.at<Vec3b>(i,j)[1] == 0) {
            redCount++;
            break;
        } 
        else {
            blackCount +=1;
            cdst.at<Vec3b>(i,j)[1] = 255;
        }
	}
    if(redCount == 0){
        rc_msg.analog_joy_a_up_down = 0;
        //steering = center;
        s = center;
    }else{
	    //steering = blackCount - center;
        rc_msg.analog_joy_a_up_down = -10;
        s = blackCount -center;
    }
    
	blackCount = 0;
    //s = steering;
	steer_output();
	//steering = 0;

    //imshow(OPENCV_WINDOW, cdst);
    //waitKey(3);
}

bool greenscreen(Mat img){
    Mat expGreen;
    cvtColor(img,expGreen, CV_BGR2GRAY );

    for(int i = 0;i<img.rows;i++){
        for(int j = 0; j<img.cols;j++){
            expGreen.at<uchar>(i,j) = ((2*img.at<Vec3b>(i,j)[1])-img.at<Vec3b>(i,j)[2]-img.at<Vec3b>(i,j)[0]);
            if(expGreen.at<uchar>(i,j) > 200){
                expGreen.at<uchar>(i,j) = 0;
            }
            if(expGreen.at<uchar>(i,j) < 50){
                expGreen.at<uchar>(i,j) = 0;
            }
        }
    }
    //imshow(OPENCV_WINDOW, expGreen);
    //waitKey(3);

    return false;
}

int main(int argc, char **argv) {
    //Setup ROS-node
    ros::init(argc, argv, "kascontrol");
    ros::NodeHandle nh;

    //Setup publisher - steering
    ros::Publisher my_msg_pub = nh.advertise<msgs::RemoteControl>("fmHMI/remote_control", 1);

    //Setup publisher rate - messages pr. sec.
    ros::Rate publish_rate(10);

    //Initiate actuation-enable signal
    enable_actuation_message();
    my_msg_pub.publish(rc_msg);

    //Camera setup
    int DEVICE = 0;
    VideoCapture cap(DEVICE);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,160);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 120);

    if(!cap.open(DEVICE)){
        cout << "VideoCapture device is not ready!\n";
        return -1;
    }

    //Time delay for the robot to setup 
    //ros::Time::init();
    //ros::Duration(2,0).sleep();
    int i = 0;
    //Message publish loop
    while(ros::ok()){

        //capture image and copy to placeholder
        cap >> frame;
    
        
        //Call Image working function
        if(i>25){
            cannyImage(frame);
            greenscreen(frame);
            i = 25;
        }else{
            rc_msg.analog_joy_a_up_down = 0;
            rc_msg.analog_joy_a_left_right = 0;
            my_msg_pub.publish(rc_msg);
        }
        i++;
        
        my_msg_pub.publish(rc_msg); //This is publishing the message
	    publish_rate.sleep();

	    ros::spinOnce();
    }
        ofstream testOutput("/home/pro/PI3test10Hz.csv");
        if(testOutput.is_open()){
            for(int j =0;j<1000;j++){
                testOutput << test[0][j] << " " << test[1][j] << "," << test[2][j] << "\n";
            }
            testOutput.close();
        }
	return 0;
}
