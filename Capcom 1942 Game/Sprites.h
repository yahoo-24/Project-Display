#ifndef SPRITES_H
#define SPRITES_H

#include "N5110.h"
extern N5110 lcd;

namespace UI {
    extern const int Zero[5][3];

    extern const int One[5][3];

    extern const int Two[5][3];

    extern const int Three[5][3];

    extern const int Four[5][3];

    extern const int Five[5][3];

    extern const int Six[5][3];

    extern const int Seven[5][3];

    extern const int Eight[5][3];

    extern const int Nine[5][3];

    void draw_number(int number, int y);

    void draw_lives(int lives);

    void draw_score(int score);

    void draw_hitpoints(int hitpoints, int max_hp);

    void separate();
}

extern const int attack_plane[7][7];

extern const int bomber_plane[8][9];

extern const int fighter_plane[6][5];

extern const int enemy_plane[3][5];

extern const int larger_enemy_plane[4][7];

extern const int extra_large_enemy_plane[5][9];

extern const int extra_large_enemy_plane_back[5][9];

extern const int extra_large_enemy_plane_right[9][5];

extern const int extra_large_enemy_plane_left[9][5];

extern const int boss_airship[11][7];

extern const int boss_plane[19][30];

extern const int bullet[2][1];

extern const int bullet_left[2][2];

extern const int bullet_right[2][2];

extern const int health[6][6];

extern const int fast_fire[6][6];

extern const int speed[6][6];

extern const int invincibility[12][13];

extern const int life[6][7];

extern const int deflect[8][6];

extern const int scatter[15][11];

extern const int support_planes[6][10];

#endif