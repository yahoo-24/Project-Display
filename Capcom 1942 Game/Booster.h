#ifndef BOOSTER_H
#define BOOSTER_H

#include "UserPlane.h"

void increase_fire_rate(UserPlane* user_plane);
void increase_speed(UserPlane* user_plane);
void regen_health(UserPlane* user_plane);
void scatter_fire_mode(UserPlane* user_plane);
void deflect_bullets(UserPlane* user_plane);
void invincible(UserPlane* user_plane);
void extra_life(UserPlane* user_plane);
void support(UserPlane* user_plane);
void power_up_isr();

class Booster {
    public:
        Booster(string type);

        void reset_position();

        bool move(set<tuple<int, int>> pixels, UserPlane* user, bool &collected);

        void draw_sprite();

        string get_type();

        int get_x();

        int get_y();

    private:
        int _position_x;
        int _position_y;
        int* _sprite;
        string _type;
        int _size_x;
        int _size_y;
        function<void(UserPlane*)> _effect;
};

void increase_speed(UserPlane* user_plane);

void increase_fire_rate(UserPlane* user_plane);

void regen_health(UserPlane* user_plane);

void scatter_fire_mode(UserPlane* user_plane);

void deflect_bullets(UserPlane* user_plane);

void invincible(UserPlane* user_plane);

void extra_life(UserPlane* user_plane);

void support(UserPlane* user_plane);

#endif