#include "Booster.h"

void increase_fire_rate(UserPlane* user_plane);
void increase_speed(UserPlane* user_plane);
void regen_health(UserPlane* user_plane);
void scatter_fire_mode(UserPlane* user_plane);
void deflect_bullets(UserPlane* user_plane);
void invincible(UserPlane* user_plane);
void extra_life(UserPlane* user_plane);
void support(UserPlane* user_plane);
void power_up_isr() { reset_plane_to_default = 1; }

Booster::Booster(string type) {
    _type = type;
    if (type == "Health") {
        _sprite = (int*) health;
        _effect = regen_health;
        _size_x = 6;
        _size_y = 6;
    } else if (type == "Fast Rate") {
        _sprite = (int*) fast_fire;
        _effect = increase_fire_rate;
        _size_x = 6;
        _size_y = 6;
    } else if (type == "Scatter") {
        _sprite = (int*) scatter;
        _effect = scatter_fire_mode;
        _size_x = 11;
        _size_y = 15;
    } else if (type == "Speed") {
        _sprite = (int*) speed;
        _effect = increase_speed;
        _size_x = 6;
        _size_y = 6;
    } else if (type == "Deflect") {
        _sprite = (int*) deflect;
        _effect = deflect_bullets;
        _size_x = 6;
        _size_y = 8;
    } else if (type == "Invincible") {
        _sprite = (int*) invincibility;
        _effect = invincible;
        _size_x = 13;
        _size_y = 12;
    } else if (type == "Life") {
        _sprite = (int*) life;
        _effect = extra_life;
        _size_x = 7;
        _size_y = 6;
    } else if (type == "Support") {
        _sprite = (int*) support_planes;
        _effect = support;
        _size_x = 10;
        _size_y = 6;
    }
    _position_x = rand() % (81 - _size_x);
    _position_y = -_size_y + 1;
}

void Booster::reset_position() {
    _position_x = rand() % 75;
    _position_y = -_size_y + 1;
}

bool Booster::move(set<tuple<int, int>> pixels, UserPlane* user, bool &collected) {
    _position_y += 1;
    bool found = false;
    // Booster is out of range
    if (_position_y > 54) {
        reset_position();
        return true;
    }
    // See if any of the user's pixels is within the booster range
    for (auto pixel : pixels) {
        int x = get<0>(pixel);
        int y = get<1>(pixel);
        if (x >= _position_x && x <= _position_x + _size_x && y >= _position_y && y <= _position_y + _size_y) {
            found = true;
            collected = true; // For the buzzer
            break;
        }
    }
    if (found) {
        _effect(user); // Run the effect
        reset_position();
        return true;
    }
    return false;
}

void Booster::draw_sprite() {
    lcd.drawSprite(_position_x, _position_y, _size_y, _size_x, _sprite);
}

string Booster::get_type() { return _type; }

int Booster::get_x() { return _position_x; }

int Booster::get_y() { return _position_y; }

void increase_speed(UserPlane* user_plane) {
    user_plane->reset_parameters();
    user_plane->set_speed_factor(1 + user_plane->get_speed_factor());
    power_up.detach();
    power_up.attach(&power_up_isr, 8s);
    power_up_led = 1;
}

void increase_fire_rate(UserPlane* user_plane) {
    user_plane->reset_parameters();
    user_plane->set_fire_rate(user_plane->get_fire_rate() / 2);
    power_up.detach();
    power_up.attach(&power_up_isr, 8s);
    power_up_led = 1;
}

void regen_health(UserPlane* user_plane) {
    user_plane->set_hitpoints(user_plane->get_max_hitpoints());
}

void scatter_fire_mode(UserPlane* user_plane) {
    user_plane->reset_parameters();
    user_plane->set_fire_mode('S');
    power_up.detach();
    power_up.attach(&power_up_isr, 10s);
    power_up_led = 1;
}

void deflect_bullets(UserPlane* user_plane) {
    user_plane->reset_parameters();
    user_plane->set_power_up("Deflect");
    power_up.detach();
    power_up.attach(&power_up_isr, 10s);
    power_up_led = 1;
}

void invincible(UserPlane* user_plane) {
    user_plane->reset_parameters();
    user_plane->set_power_up("Invincible");
    power_up.detach();
    power_up.attach(&power_up_isr, 8s);
    power_up_led = 1;
}

void extra_life(UserPlane* user_plane) {
    user_plane->add_life();
}

void support(UserPlane* user_plane) {
    user_plane->set_support(true);
}