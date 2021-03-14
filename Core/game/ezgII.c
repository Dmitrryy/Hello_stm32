/*************************************************************************************************
 *
 *   ezgII.c
 *
 *   Created by dmitry
 *   14.03.2021
 *
 ***/

#include "game.h"


int scanLine(const int line[EZG_DISPLAY_SIZE_X])
{
    int res = -1;
    for (int i = 0; i < EZG_DISPLAY_SIZE_X; ++i)
    {
        if (line[i] == 1) {
            res = i;
            break;
        }
    }

    return res;
}


void ezgIIPlay(int display[EZG_DISPLAY_SIZE_Y][EZG_DISPLAY_SIZE_X])
{
    if (ezgGetStatus() == EZG_STATUS_GAME)
    {
        for (int i = EZG_DISPLAY_SIZE_Y - 2; i >= 0; --i)
        {
            int res_scan_line = scanLine(display[i]);
            if (res_scan_line != -1) {
                const float newPosition = res_scan_line - ezgGetPlayerWidth() / 2.f;
                ezgSetPlayerPosition((newPosition >= 0) ? newPosition : 0.f);
                break;
            }
        }
    }
}