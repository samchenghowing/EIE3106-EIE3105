/*
 * main.cpp
 *
 *  Created on: 2 May, 2017
 *      Author: sylam
 */

#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "command.h"
#include "Object.h"

using namespace cv;
using namespace std;

Rect roi = Rect(330,70,620,620);

extern ostream udp;

void drawClock(void);

namespace
{

bool selectObject = false;
Rect selection;
void onMouse( int event, int x, int y, int, void *p)
{
	Mat *mat = (Mat*)p;
	x = MAX(x, 0); y = MAX(y, 0);
	x = MIN(x, mat->cols); y = MIN(y, mat->rows);
	static Point anchor;
    if( selectObject )
    {
        selection.x = MIN(x, anchor.x);
        selection.y = MIN(y, anchor.y);
        selection.width = abs(x - anchor.x);
        selection.height = abs(y - anchor.y);
    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
    	anchor = Point(x,y);
        selection = Rect(x,y,0,0);
        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        break;
    }
}

void onMouseHS(int event, int x, int y, int, void*){
	cout << x/2 << " : " << y/2 << endl;
}

Mat histogram(Mat hsv) {
    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 256, sbins = 256;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 256 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    Mat hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    calcHist( &hsv, 1, channels, Mat(), // do not use mask
             hist, 2, histSize, ranges,
             true, // the histogram is uniform
             false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 2;
    Mat histImg = Mat::zeros(sbins*scale, hbins*scale, CV_8UC3);

    for( int h = 0; h < hbins; h++ )
        for( int s = 0; s < sbins; s++ )
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);
            rectangle( histImg, Point(h*scale, s*scale),
                        Point( (h+1)*scale - 1, (s+1)*scale - 1),
                        Scalar::all(intensity),
                        CV_FILLED );
        }
    return histImg;
}

void overlay(Mat bgr, Mat hsv, Mat bg) {
	Mat alpha, beta, temp[3];
	split(hsv, temp);
	threshold(temp[1], alpha, 128, 255, THRESH_BINARY);
	beta = Mat::zeros(alpha.size(), CV_8UC1);
	beta = ~alpha;
	split(bg, temp);
	temp[0] &= alpha;
	temp[1] &= alpha;
	temp[2] &= alpha;
	merge(temp, 3, alpha);
	split(bgr, temp);
	temp[0] &= beta;
	temp[1] &= beta;
	temp[2] &= beta;
	merge(temp, 3, beta);
	bgr = alpha | beta;
}

std::vector<cv::Point> cmd_params;
command cmd = IDLE;
int cmd_cnt;

}//anonymous

int main(int argc, char **argv) {
	Ball ballBlue("BBE", Scalar(100,135,50), Scalar(106,174,224));
	Ball ballOrange("BOE", Scalar(18,200,50), Scalar(22,255,224));
	Ball ballPink("BPK", Scalar(170,200,50), Scalar(177,250,224));
	Ball ballYellow("BYW", Scalar(29,160,50), Scalar(32,244,224));
	Ball ballRed("BRD", Scalar(177,179,50), Scalar(181,253,224));
	Car carYellow("CYW", Scalar(26,74,50), Scalar(34,249,224));
	Car carBlue("CBE", Scalar(105,172,50), Scalar(117,240,250));
//	Car carRed("CRD", Scalar(175,179,50), Scalar(185,253,224));
	Car carViolet("CVT", Scalar(118,130,50), Scalar(128,160,224));

    VideoCapture capture(0); // open the video camera no. 0
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);	//1280, 800
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);	//760,600
//    VideoCapture capture("lego1.avi");

    if (!capture.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    Mat frame, hsv, hsv_roi;
    Mat bg = imread("ring.jpg", CV_LOAD_IMAGE_COLOR);

    int frameNo = 0;

    while (waitKey(1)!=27) {
    	frameNo++;
    	if (cmd_cnt) if(!--cmd_cnt) cmd = IDLE;
    	capture >> frame;
    	if (frame.empty()) break;
    	cvtColor(frame, hsv, COLOR_BGR2HSV);
    	hsv_roi = hsv(roi);
//    	overlay(frame, hsv, bg);
    	Object::search3draw(frame, hsv_roi, frameNo);
		udp << "CMD" << setw(3) << (frameNo & 0xfff);
		udp << setw(6) << cmd;
		for (auto &obj : cmd_params) {
			udp << setw(3) << obj.x;
			udp << setw(3) << obj.y;
		}
		udp << endl;
    	if (selectObject) {
    		rectangle(frame, selection, Scalar(0,0,255), 1, 8);
    		imshow("histogram:h-s", histogram(hsv(selection)));
    	    setMouseCallback( "histogram:h-s", onMouseHS, 0 );
    	}
        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        setMouseCallback( "MyVideo", onMouse, &frame );
        drawClock();
    }

	return 0;
}

void send_command(command c) {
	cmd_cnt = 20;
	cmd = c;
}


