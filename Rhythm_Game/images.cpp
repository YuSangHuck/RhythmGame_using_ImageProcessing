#include "rhythm_game.h"

using namespace cv;

void Rhythm_Game::image()
{
	QString url = R"(D:\Users\images\startButtonBasic.png)";
	QPixmap img(url);
	ui.label->setPixmap(img);
}
