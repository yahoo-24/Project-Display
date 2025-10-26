#ifndef USERPLANE_H
#define USERPLANE_H

#include <tuple>
#include <string>
#include <set>
#include <map>
#include "Sprites.h"
#include "Globals.h"
#include "mbed.h"

class UserPlane {
    public:
        int* sprite;

        UserPlane(int hp, int h, int w);
        virtual ~UserPlane();

        virtual void fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float);

        virtual void reset_parameters();

        virtual void special_ability(char &type);

        void lost_life();

        bool plane_hit(int hp_lost);

        bool game_over();

        int get_points();

        int get_hitpoints();

        int get_max_hitpoints();

        int get_lives();

        float get_speed_factor();

        int get_height();

        int get_width();

        char get_fire_mode();

        chrono::milliseconds get_fire_rate();

        void set_fire_rate(chrono::milliseconds rate);

        void set_speed_factor(float factor);

        void set_hitpoints(int hp);

        void set_fire_mode(char mode);

        void score_points(int points);

        void add_life();

        void set_power_up(string power_up);

        string get_power_up();

        void set_support(bool support);

        bool get_support();

        void reset_points();

    protected:
        int _hitpoints;
        int _max_hp;
        int _lives;
        int _points;
        float _speed_factor;
        int _height;
        int _width;
        char _fire_mode;
        string _power_up = "None";
        chrono::milliseconds _fire_rate;
        bool _support = false;
};

class FighterPlane : public UserPlane {
    public:
        FighterPlane();

        void fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) override;

        void reset_parameters() override;

        void special_ability(char &type) override;

        void set_active();

        bool get_active();

        void set_position(int x, int y);

        tuple<int, int> get_position();

        void draw_plane();

        bool check_collision(tuple<int, int> target, bool is_bullet);

        void update_pixels();

        void reset_plane();

        void support_fire(map<char, set<tuple<float, float>>> &bullet_set);
    
    private:
        bool _active = false;
        int _position_x;
        int _position_y;
        set<tuple<int, int>> _pixels;
};

class AttackingPlane : public UserPlane {
    public:
        AttackingPlane();

        void fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) override;

        void reset_parameters() override;

        void special_ability(char &type) override;
};

class Bomber : public UserPlane {
    public:
        Bomber();

        void fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) override;

        void reset_parameters() override;

        void special_ability(char &type) override;
};

#endif