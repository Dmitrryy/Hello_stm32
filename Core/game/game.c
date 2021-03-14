/*************************************************************************************************
 *
 *   game.c
 *
 *   Created by dmitry
 *   12.03.2021
 *
 ***/

#include "game.h"
#include <stdlib.h>




/****************************************************************************************
 *
 *   structures of game entities
 *
 ***/
struct ezgIPair
{
    int x, y;
};
struct ezgFPair
{
    float x, y;
};

struct ezgBall
{
    struct ezgFPair position;
    float           speed;
    int             coin;

    int             alive;
};

struct ezgPlayerPlatform
{
    struct ezgFPair position;
    int             width;
    int             coins;
};
/***
 *
 *   end structures of game entities
 *
 ****************************************************************************************/




/****************************************************************************************
 *
 *   global variables
 *
 ***/
#define EZG_MAX_BALLS  50
#define EZG_TIME_BETWEEN_SPAWN 1

struct ezgBall           g_balls[EZG_MAX_BALLS];
size_t                   g_curBallsCount = 0;

struct ezgPlayerPlatform g_player = {};

float                    g_gameTime = 0;
float                    g_timeNextSpawn = 1;

enum ezgGameStatus       g_status = EZG_STATUS_NOT_STARTED;


int g_display[EZG_DISPLAY_SIZE_X][EZG_DISPLAY_SIZE_Y] = {
        { 1, 0, 1, 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0, 1, 0, 1 },
        { 1, 0, 1, 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0, 1, 0, 1 },
        { 1, 0, 1, 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0, 1, 0, 1 },
        { 1, 0, 1, 0, 1, 0, 1, 0 },
        { 0, 1, 0, 1, 0, 1, 0, 1 }
};
/***
 *
 *   end of global variables
 *
 ****************************************************************************************/




/****************************************************************************************
 *
 *   helper prototypes
 *
 ***/
void clearDisplay    ();

void displayEntities();
void displayBall    (struct ezgBall ball);
void displayPlayer  (struct ezgPlayerPlatform);

void updateBall  (struct ezgBall* ball, float dt);
void spawnBall   ();
void updatePlayer(struct ezgPlayerPlatform* player, float dt); //TODO

int checkIntersectionBallPlayer(struct ezgBall ball, struct ezgPlayerPlatform player);
/***
 *
 *   end of helper prototypes
 *
 ****************************************************************************************/




enum ezgGameStatus ezgGetStatus()
{
    return g_status;
}


size_t ezgGetCoins()
{
    return g_player.coins;
}


void ezgStart()
{
    g_status = EZG_STATUS_GAME;

    g_curBallsCount = 0;

    g_player.coins = 0;
    g_player.position.x = EZG_DISPLAY_SIZE_X / 2.f - 1;
    g_player.position.y = EZG_DISPLAY_SIZE_Y - 1;
    g_player.width = 4;

    g_gameTime = 0;
}


int** ezgGetDisplayMatrix()
{
    clearDisplay();
    displayEntities();

    return g_display;
}


void ezgUpdate (float dt)
{
    if (g_status == EZG_STATUS_GAME)
    {
        //updatePlayer(&g_player, dt); //TODO

        for(size_t i = 0; i < g_curBallsCount; ++i)
        {
            updateBall(g_balls + i, dt);
        }


        for (size_t i = 0; i < g_curBallsCount; ++i)
        {
            if (checkIntersectionBallPlayer(g_balls[i], g_player))
            {
                g_balls[i].alive = 0;
                g_player.coins += g_balls[i].coin;
            }
        }


        if (g_gameTime + dt >= g_timeNextSpawn) {
            spawnBall();
            g_timeNextSpawn += EZG_TIME_BETWEEN_SPAWN;
        }


        for(size_t i = 0; i < g_curBallsCount; ++i)
        {
            if(!g_balls[i].alive) {
                g_balls[i] = g_balls[g_curBallsCount - 1];
                g_curBallsCount--;
                i--;
            }
        }
    }
    g_gameTime += dt;
}

void ezgSetPlayerPosition (float new_x)
{
    g_player.position.x = new_x;
}


/****************************************************************************************
 *
 *   defining helper functions
 *
 ***/
void clearDisplay()
{
    for (size_t y = 0; y < EZG_DISPLAY_SIZE_Y; ++y)
    {
        for (size_t x = 0; x < EZG_DISPLAY_SIZE_X; ++x)
        {
            g_display[y][x] = 0;
        }
    }
}


void spawnBall()
{
    if (g_curBallsCount >= EZG_MAX_BALLS) {
        return;
    }


    do {
        g_balls[g_curBallsCount].position.x = rand() % 10;
    } while(g_balls[g_curBallsCount].position.x >= EZG_DISPLAY_SIZE_X);

    g_balls[g_curBallsCount].position.y = 0.f;
    g_balls[g_curBallsCount].coin = 2;
    g_balls[g_curBallsCount].speed = 4;
    g_balls[g_curBallsCount].alive = 1;

    g_curBallsCount++;
}


void updateBall (struct ezgBall* ball, float dt)
{
    ball->position.y += dt * ball->speed;

    if(ball->position.y >= EZG_DISPLAY_SIZE_Y)
    {
        ball->alive = 0;
    }

}


void displayEntities ()
{
    for(size_t i = 0; i < g_curBallsCount; ++i)
    {
        displayBall(g_balls[i]);
    }

    displayPlayer(g_player);
}


void displayBall (struct ezgBall ball)
{
    if (ball.position.x < 0 || ball.position.x >= EZG_DISPLAY_SIZE_X ||
        ball.position.y < 0 || ball.position.y >= EZG_DISPLAY_SIZE_Y)
    {
        return ;
    }

    g_display[(int)ball.position.y][(int)ball.position.x] = 1;
}


void displayPlayer  (struct ezgPlayerPlatform player)
{
    if (player.position.x < 0 || player.position.x >= EZG_DISPLAY_SIZE_X  ||
        player.position.y < 0 || player.position.y >= EZG_DISPLAY_SIZE_Y)
    {
        return ;
    }
    for (int i = 0; i < player.width; ++i)
    {
        g_display[(int)player.position.y][((int)player.position.x + i) % EZG_DISPLAY_SIZE_X] = 1;
    }
}


int checkIntersectionBallPlayer(struct ezgBall ball, struct ezgPlayerPlatform player)
{
    int res = 0;
    if ((int)ball.position.y == (int)player.position.y)
    {
        for (int i = 0; i < player.width; ++i)
        {
            if ((int)ball.position.x == ((int)player.position.x + i) % EZG_DISPLAY_SIZE_X) {
                res = 1;
                break;
            }
        }
    }

    return res;
}
/***
 *
 *   end of defining helper functions
 *
 ****************************************************************************************/