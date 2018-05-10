#include "rhythm_game.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Rhythm_Game w;
	w.show();
	return a.exec();
}
