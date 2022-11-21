/*
 * Object.cpp
 *
 *  Created on: 3 May, 2017
 *      Author: sylam
 */

#include <iomanip>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "Object.h"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

extern Rect roi;
extern ostream udp;

Mat Object::noiseElement = getStructuringElement( MORPH_RECT,Size(2,2));
Mat Object::fillElement = getStructuringElement( MORPH_RECT,Size(6,6));
Mat Object::ballMask;

Object *Object::balls, *Object::cars;

Object::Object(const char *n, Scalar mn, Scalar mx, int s) {
	name = n; size = s; min = mn; max = mx;
}

void Object::search3draw(Mat frame, Mat hsv, int frameNo) {
	ballMask = Mat::zeros(roi.size(), CV_8UC1);
	ballMask.setTo(255);
	Object *p = balls;
	while (p) {
    	p->search(hsv);
    	p->draw(frame, frameNo);
		p = p->next;
	}
//	imshow("mask", ballMask);
	p = cars;
	while (p) {
    	p->search(hsv);
    	p->draw(frame, frameNo);
		p = p->next;
	}
}

void Object::draw(Mat mat, int frameNo) {
	if (!objs.empty()) {
		udp << name << setw(3) << (frameNo & 0xfff);
//		circle(mat, Point(objs[0].x, objs[0].y), size>>1, Scalar(0,0,255), 3, 8);
		for (auto &obj : objs) {
			circle(mat, Point(obj.x, obj.y), size, Scalar(0,0,255), 2, 8);
			udp << setw(3) << obj.x;
			udp << setw(3) << obj.y;
		}
		udp << endl;
	}
}

Ball::Ball(const char *n, Scalar mn, Scalar mx)
: Object(n, mn, mx, 10) {
	next = balls;
	balls = this;
}

Mat Ball::search(Mat hsv) {
	objs.clear();
	Mat threshold;
	inRange(hsv, min, max, threshold);
    erode(threshold, threshold, noiseElement);
	dilate(threshold, threshold, fillElement);
//	imshow(name, threshold);
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (hierarchy.size() > 0) {
    	int area = size * size * 2.5f;
    	int area1 = size * size * 7;
    	int count = 5;					// set limit
        for (int index = 0; index >= 0; index = hierarchy[index][0]) {
            Moments moment = moments((Mat)contours[index]);
            if ((moment.m00 > area) && (moment.m00 < area1))
            	if (count--)			// check limit
            	{
            		int x = moment.m10/moment.m00;
            		int y = moment.m01/moment.m00;
            		circle(ballMask, Point(x, y), size+4, Scalar(0,0,0),-1,8);
					objs.push_back(Point3i(
							x + roi.x,
							y + roi.y,
							moment.m00
							));
            	}
        }
        sort(objs.begin(), objs.end(), [](Point3i a, Point3i b){
//        	return a.z > b.z;	// descending
        	return a.z < b.z;	// ascending
        });
    }
    return threshold;
}

Car::Car(const char *n, Scalar mn, Scalar mx)
: Object(n, mn, mx, 7) {
	next = cars;
	cars = this;
}

Mat Car::search(Mat hsv) {
	objs.clear();
	Mat threshold;
	inRange(hsv, min, max, threshold);
	threshold &= ballMask;
    erode(threshold, threshold, noiseElement);
	dilate(threshold, threshold, fillElement);
//	imshow(name, threshold);
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    if (hierarchy.size() > 0) {
    	int area = size * size * 2;
    	int area1 = size * size * 7;
    	int count = 5;					// set limit
        for (int index = 0; index >= 0; index = hierarchy[index][0]) {
            Moments moment = moments((Mat)contours[index]);
            if ((moment.m00 > area) && (moment.m00 < area1))
            	if (count--)			// check limit
					objs.push_back(Point3i(
							moment.m10/moment.m00 + roi.x,
							moment.m01/moment.m00 + roi.y,
							moment.m00
							));
        }
    }
    return threshold;
}
