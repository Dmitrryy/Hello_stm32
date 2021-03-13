/*************************************************************************************************
 *
 *   game.h
 *
 *   Created by dmitry
 *   12.03.2021
 *
 ***/

#pragma once

#include <stdlib.h>

#define EZG_DISPLAY_SIZE_X 8
#define EZG_DISPLAY_SIZE_Y 8


enum ezgGameStatus
{
    EZG_STATUS_GAME
    , EZG_STATUS_PAUSE
    , EZG_STATUS_NOT_STARTED
};


void ezgStart();

enum ezgGameStatus ezgGetStatus();
size_t             ezgGetCoins ();


int** ezgGetDisplayMatrix();

void ezgUpdate(float dt);
void ezgSetPlayerPosition(float new_x);


