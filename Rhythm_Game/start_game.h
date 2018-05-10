#ifndef START_GAME_H
#define START_GAME_H

#include "rhythm_game.h"

class Start_Game : public QMainWindow
{
	Q_OBJECT

public:
	Start_Game(QWidget *parent = 0);
	~Start_Game();
	void start_game();

private:
	Ui::Rhythm_GameClass ui;

	private slots:
};

#endif // START_GAME_H
