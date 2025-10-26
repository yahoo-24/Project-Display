#ifndef BOSSPLANE_H
#define BOSSPLANE_H

#include "Enemies.h"
#include "UserPlane.h"

class BossPlane {
    public:
        BossPlane(short type);

        bool get_defeated();

        void boss_plane_pixels();

        void draw_explosion();

        int damaged(map<char, set<tuple<float, float>>> &bullet_set);

        bool destroyed(map<char, set<tuple<float, float>>> &bullet_set);

        bool check_collision(set<tuple<int, int>> &user_pixels, UserPlane* plane);

        bool move(set<tuple<int, int>> &user_pixels, UserPlane* plane);

        void fire_rounds(float mx, float my);

        bool update_fire(set<tuple<int, int>> &user_pixels, UserPlane* plane);

    private:
        int _hp;
        float _position_x;
        float _position_y;
        set<tuple<int, int>> _boss_pixels;
        set<EnemyBullet> _boss_bullets;
        bool _defeated;
        short _type;
};

#endif