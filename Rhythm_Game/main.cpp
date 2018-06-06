#include "rhythm_game.h"
#include "hand.hpp"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
//	QApplication a(argc, argv);
//	Rhythm_Game w;
//	w.show();
//	return a.exec();
//
	namedWindow("main");
	moveWindow("main", 10, 10);
	Cap cap;
	Mat* frame;
	while (waitKey(1) != 27){
		frame = cap.ReadCam();
		imshow("main", *frame);
	}
}

