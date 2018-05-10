#ifndef RHYTHM_GAME_H
#define RHYTHM_GAME_H

#include <QtWidgets/QMainWindow>
#include <QTimer>

#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\core.hpp"

#include "ui_rhythm_game.h"

class Rhythm_Game : public QMainWindow
{
	Q_OBJECT

public:
	Rhythm_Game(QWidget *parent = 0);
	~Rhythm_Game();
	void image();

private:
	Ui::Rhythm_GameClass ui;
	QTimer * timer;

private slots:
	void start_button();
	void exit_button();
	void update_picture();
};

#endif // RHYTHM_GAME_H
