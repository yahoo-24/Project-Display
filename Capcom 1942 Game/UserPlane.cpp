#include "UserPlane.h"

UserPlane::UserPlane(int hp, int h, int w) : _lives{3}, _hitpoints{hp}, _max_hp{hp}, _points{0},
_height{h}, _width{w}, _fire_mode('n') {};
UserPlane::~UserPlane() {}

void UserPlane::fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) {}

void UserPlane::reset_parameters() {}

void UserPlane::special_ability(char &type) {}

void UserPlane::lost_life() {
    _lives -= 1;
    _hitpoints = _max_hp;
}

bool UserPlane::plane_hit(int hp_lost) {
    if (hp_lost >= _hitpoints) {
        lost_life();
        return 1;
    } else {
        _hitpoints -= hp_lost;
        return 0;
    }
}

bool UserPlane::game_over() { return _lives == 0; }

int UserPlane::get_points() { return _points; }

int UserPlane::get_hitpoints() { return _hitpoints; }

int UserPlane::get_max_hitpoints() { return _max_hp; }

int UserPlane::get_lives() { return _lives; }

float UserPlane::get_speed_factor() { return _speed_factor; }

int UserPlane::get_height() { return _height; }

int UserPlane::get_width() { return _width; }

char UserPlane::get_fire_mode() { return _fire_mode; }

chrono::milliseconds UserPlane::get_fire_rate() { return _fire_rate; }

void UserPlane::set_fire_rate(chrono::milliseconds rate) { _fire_rate = rate; }

void UserPlane::set_speed_factor(float factor) { _speed_factor = factor; }

void UserPlane::set_hitpoints(int hp) { _hitpoints = hp; }

void UserPlane::set_fire_mode(char mode) { _fire_mode = mode; }

void UserPlane::score_points(int points) { _points += points; }

void UserPlane::add_life() { _lives++; }

void UserPlane::set_power_up(string power_up) { _power_up = power_up; }

string UserPlane::get_power_up() { return _power_up; }

void UserPlane::set_support(bool support) { _support = support; }

bool UserPlane::get_support() { return _support; }

void UserPlane::reset_points() { _points = 0; }


FighterPlane::FighterPlane() : UserPlane(60, 6, 5) {
    _speed_factor = 1.3;
    _fire_rate = 200ms;
    sprite = (int *)fighter_plane;
}

void FighterPlane::fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) {
    bullet_set_float['S'].insert(make_tuple(x_pos + 2, y_pos));
    if (_fire_mode == 'S') {
        bullet_set_float['L'].insert(make_tuple(x_pos + 2, y_pos));
        bullet_set_float['R'].insert(make_tuple(x_pos + 2, y_pos));
    }
}

void FighterPlane::reset_parameters() {
    _speed_factor = 1.3;
    _fire_mode = 'n';
    _fire_rate = 200ms;
    _power_up = "None";
}

void FighterPlane::special_ability(char &type) {
    type = 'F';
}

void FighterPlane::set_active() { _active = true; }

bool FighterPlane::get_active() { return _active; }

void FighterPlane::set_position(int x, int y) {
    _position_x = x;
    _position_y = y;
    update_pixels();
    draw_plane();
}

tuple<int, int> FighterPlane::get_position() { return make_tuple(_position_x, _position_y); }

void FighterPlane::draw_plane() {
    lcd.drawSprite(_position_x, _position_y, 6, 5, (int *)fighter_plane);
}

bool FighterPlane::check_collision(tuple<int, int> target, bool is_bullet) {
    if (!_active) {
        return false;
    }
    // See if the target is in the pixels array and then check if it is a bullet or a plane collision
    bool hit = _pixels.find(target) != _pixels.end();
    if (hit && is_bullet) {
        _hitpoints -= 20;
        _active = (_hitpoints > 0) ? true : false;
    } else if (hit && !is_bullet) {
        _active = false;
    }
    return hit;
}

void FighterPlane::update_pixels() {
    _pixels.clear();
    if (_active) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 5; j++) {
                if (fighter_plane[i][j] == 1) {
                    _pixels.insert(make_tuple(_position_x + j, _position_y + i));
                }
            }
        }
    }
}

void FighterPlane::reset_plane() {
    _hitpoints = 60;
    set_active();
}

void FighterPlane::support_fire(map<char, set<tuple<float, float>>> &bullet_set) {
    if (_active) {
        bullet_set['S'].insert(make_tuple(_position_x + 2, _position_y));
    }
}


AttackingPlane::AttackingPlane() : UserPlane(100, 7, 7) {
    _speed_factor = 1;
    _fire_rate = 400ms;
    sprite = (int *)attack_plane;
}

void AttackingPlane::fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) {
    bullet_set_float['S'].insert(make_tuple(x_pos + 2, y_pos));
    bullet_set_float['S'].insert(make_tuple(x_pos + 4, y_pos));
    if (_fire_mode == 'S') {
        bullet_set_float['L'].insert(make_tuple(x_pos + 2, y_pos));
        bullet_set_float['R'].insert(make_tuple(x_pos + 4, y_pos));
    }
}

void AttackingPlane::reset_parameters() {
    _speed_factor = 1;
    _fire_mode = 'n';
    _fire_rate = 400ms;
    _power_up = "None";
}

void AttackingPlane::special_ability(char &type) {
    type = 'A';
}


Bomber::Bomber() : UserPlane(200, 8, 9) {
    sprite = (int *)bomber_plane;
    _fire_rate = 600ms;
    _speed_factor = 0.85;
}

void Bomber::fire_weapon(map<char, set<tuple<float, float>>> &bullet_set_float) {
    bullet_set_float['S'].insert(make_tuple(x_pos + 2, y_pos));
    bullet_set_float['S'].insert(make_tuple(x_pos + 4, y_pos - 1));
    bullet_set_float['S'].insert(make_tuple(x_pos + 6, y_pos - 1));
    if (_fire_mode == 'S') {
        bullet_set_float['L'].insert(make_tuple(x_pos + 2, y_pos));
        bullet_set_float['R'].insert(make_tuple(x_pos + 2, y_pos));
    }
}

void Bomber::reset_parameters() {
    _speed_factor = 0.7;
    _fire_mode = 'n';
    _fire_rate = 600ms;
    _power_up = "None";
}

void Bomber::special_ability(char &type) {
    type = 'B';
}