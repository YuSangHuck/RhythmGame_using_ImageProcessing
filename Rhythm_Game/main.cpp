#include "rhythm_game.h"
#include <QtWidgets/QApplication>
///PR test - change
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Rhythm_Game w;
	w.show();
	return a.exec();
}
