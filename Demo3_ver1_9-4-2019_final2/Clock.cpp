/*
 * Clock.cpp
 *
 *  Created on: 9 May, 2017
 *      Author: sylam
 */

#include <iostream>
#include <chrono>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "command.h"

using namespace cv;
using namespace std;

void send_command(command);

namespace
{

    Point mousePoint;
    int mouseEvent;
    
    void onMouse(int event, int x, int y, int, void *p) {
//        cout << x << " : " << y << endl;
        mousePoint = Point(x, y);
        mouseEvent = event;
    }
    
    class Button: public Rect {
    public:
        Button(command c, int x, int y, int w, int h): Button(x, y, w, h) {
            cmd = c;
        }
        Button(int x, int y, int w, int h): Rect(x,y,w,h){
            lastLeft = true;
        }
        int mouse(Mat mat, int state) {
            if (contains(mousePoint)) {
                rectangle(mat, Point(x,y), Point(x+width, y+height), Scalar(0,0,100), 2, 8);
                if (mouseEvent==EVENT_LBUTTONUP) lastLeft = true;
                if ((mouseEvent==EVENT_LBUTTONDOWN)&&(lastLeft)) {
                    lastLeft = false;
                    state = feedback(state);
                }
            } else lastLeft = true;
            draw(mat, state);
            return state;
        }
    private:
        bool lastLeft;
        command cmd;
        virtual int feedback(int) {
            send_command(cmd);
            return 0;
        }
        virtual void draw(Mat, int){}
    };
    
    class ClrButton: public Button {
    public:
        ClrButton(int x, int y, int w, int h): Button(x,y,w,h){}
    private:
        virtual int feedback(int i) {
            send_command(RESET);
            return RESET;
        }
    };
    
    class RunStop: public Button {
    public:
        RunStop(int x, int y, int w, int h): Button(x,y,w,h){}
    private:
        virtual int feedback(int i) {
            i = i == RUN ? STOP : RUN;
            if (i == RUN) send_command(RUN);
            if (i == STOP) send_command(STOP);
            return i;
        }
        void draw(Mat mat, int state){
            const char *prompt = state == RUN ? "STOP" : "RUN";
            putText(mat, prompt, Point(x+30,y+40), FONT_HERSHEY_COMPLEX_SMALL, 2.0, Scalar(), 2, CV_AA);
        }
    };
    
    class Clock {
    public:
        Clock(void):
        runStop(321,131,167,57),
        b_clear(312,197,185,30),
        b0(ZERO, 17, 187, 41, 32),
        b1(ONE, 76, 187, 41, 32),
        b2(TWO, 135, 187, 41, 32),
        b3(THREE, 194, 187, 41, 32),
        b4(FOUR, 254, 187, 41, 32),
        b5(FIVE, 17, 131, 41, 32),
        b6(SIX, 76, 131, 41, 32),
        b7(SEVEN, 135, 131, 41, 32),
        b8(EIGHT, 194, 131, 41, 32),
        b9(NINE, 254, 131, 41, 32)
        {
            state = RESET;
            image = imread("clock.png");
            namedWindow("Clock");
            setMouseCallback("Clock", onMouse, 0);
        }
        void draw(void);
    private:
        RunStop runStop;
        ClrButton b_clear;
        Button b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;
        Mat image;
        volatile int state;
        chrono::time_point<chrono::system_clock, chrono::duration<double> > start;
        void clock(void);
    } ck;
    
    void Clock::draw(void) {
        static chrono::time_point<chrono::system_clock, chrono::duration<double> > cur;
        static chrono::duration<double> elapsed_seconds;
        Mat mat = image.clone();
        cur = chrono::system_clock::now();
        switch (state) {
            case RESET:
                start = cur;
                break;
            case STOP:
                start = cur - elapsed_seconds;
                // no break
            default:
                break;
        }
        elapsed_seconds = cur - start;
        putText(mat, to_string(elapsed_seconds.count()), Point(40,90), FONT_HERSHEY_COMPLEX_SMALL, 4.0, Scalar(), 5, CV_AA);
        state = runStop.mouse(mat, state);
        state = b_clear.mouse(mat, state);
        b0.mouse(mat, 0);
        b1.mouse(mat, 0);
        b2.mouse(mat, 0);
        b3.mouse(mat, 0);
        b4.mouse(mat, 0);
        b5.mouse(mat, 0);
        b6.mouse(mat, 0);
        b7.mouse(mat, 0);
        b8.mouse(mat, 0);
        b9.mouse(mat, 0);
        imshow("Clock", mat);
    }
    
}//anonymous

void drawClock(void) { ck.draw(); }

