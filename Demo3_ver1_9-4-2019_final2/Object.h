/*
 * Object.h
 *
 *  Created on: 3 May, 2017
 *      Author: sylam
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include <opencv2/core/core.hpp>

class Object {
public:
	Object(const char *name, cv::Scalar, cv::Scalar, int size);
//	virtual ~Object(void);
	static void search3draw(cv::Mat frame, cv::Mat hsv, int frameNo);
protected:
	const char *name;
	std::vector<cv::Point3i> objs;
	int size;
	cv::Scalar min, max;
	static cv::Mat noiseElement, fillElement, ballMask;
	static Object *balls, *cars;
	Object *next;
private:
	virtual cv::Mat search(cv::Mat)=0;
	void draw(cv::Mat, int frameNo);
};

class Ball: public Object {
public:
	Ball(const char *name, cv::Scalar, cv::Scalar);
private:
	cv::Mat search(cv::Mat);
};

class Car: public Object {
public:
	Car(const char *name, cv::Scalar, cv::Scalar);
private:
	cv::Mat search(cv::Mat);
};

#endif /* OBJECT_H_ */
