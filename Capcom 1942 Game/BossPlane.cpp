#include "BossPlane.h"

BossPlane::BossPlane(short type) : _hp{2000} {
    _position_x = 36;
    _position_y = -7;
    _boss_bullets = {};
    _defeated = false;
    _type = type;
    switch (type) {
        case 1:
            _hp = 2000;
            break;
        case 2:
            _hp = 2500;
            break;
        case 3:
            _hp = 3000;
            break;
        case 4:
            _hp = 3500;
            break;
        case 5:
            _hp = 4500;
            _position_x = 24;
            _position_y = 48;
            break;
    }
}

bool BossPlane::get_defeated() { return _defeated; }

void BossPlane::boss_plane_pixels() {
    // Update pixels
    _boss_pixels.clear();
    if (_type != 5) {
        for (int i = 0; i < 11; i++) {
            for (int j = 0; j < 7; j++) {
                if (boss_airship[i][j] == 1) {
                    _boss_pixels.insert(make_tuple(_position_x + j, _position_y + i));
                }
            }
        }
    } else {
        for (int i = 0; i < 19; i++) {
            for (int j = 0; j < 30; j++) {
                if (boss_plane[i][j] == 1) {
                    _boss_pixels.insert(make_tuple(_position_x + j, _position_y + i));
                }
            }
        }
    }
}

void BossPlane::draw_explosion() {
    lcd.drawCircle(_position_x, _position_y, 15, FILL_BLACK);
}

int BossPlane::damaged(map<char, set<tuple<float, float>>> &bullet_set) {
    int damage = 0;
    set<tuple<float, float>> bullets_to_remove = {};
    for (tuple<float, float> bullet : bullet_set['S']) {
        tuple<int, int> int_bullet = make_tuple(get<0>(bullet), get<1>(bullet));
        if (_boss_pixels.find(int_bullet) != _boss_pixels.end()) {
            damage += 100;
            bullets_to_remove.insert(bullet);
        }
    }
    for (tuple<float, float> bullet : bullets_to_remove) {
        bullet_set['S'].erase(bullet);
    }
    bullets_to_remove.clear();
    for (tuple<float, float> bullet : bullet_set['R']) {
        tuple<int, int> int_bullet = make_tuple(get<0>(bullet), get<1>(bullet));
        if (_boss_pixels.find(int_bullet) != _boss_pixels.end()) {
            damage += 100;
            bullets_to_remove.insert(bullet);
        }
    }
    for (tuple<float, float> bullet : bullets_to_remove) {
        bullet_set['R'].erase(bullet);
    }
    bullets_to_remove.clear();
    for (tuple<float, float> bullet : bullet_set['L']) {
        tuple<int, int> int_bullet = make_tuple(get<0>(bullet), get<1>(bullet));
        if (_boss_pixels.find(int_bullet) != _boss_pixels.end()) {
            damage += 100;
            bullets_to_remove.insert(bullet);
        }
    }
    for (tuple<float, float> bullet : bullets_to_remove) {
        bullet_set['L'].erase(bullet);
    }
    return damage;
}

bool BossPlane::destroyed(map<char, set<tuple<float, float>>> &bullet_set) {
    _hp -= damaged(bullet_set);
    if (_hp < 0) {
        _defeated = true;
        draw_explosion();
        return true;
    }
    return false;
}

bool BossPlane::check_collision(set<tuple<int, int>> &user_pixels, UserPlane* plane) {
    for (tuple<int, int> pixel : user_pixels) {
        if (_boss_pixels.find(pixel) != _boss_pixels.end()) {
            plane->lost_life();
            lcd.drawCircle(x_pos, y_pos, 5, FILL_BLACK);
            // Moves the plane to a different place away from the boss
            // This prevents the user losing multiple lives due to collision
            x_pos = 0;
            y_pos = 40;
            return true;
        }
    }
    return false;
}

bool BossPlane::move(set<tuple<int, int>> &user_pixels, UserPlane* plane) {
    if (_type != 5) {
        _position_y += (_position_y < 20) ? 0.2 : 0;
        _position_x += (x_pos > _position_x) ? 0.2 : -0.2;
    } else {
        _position_y -= (_position_y > 8) ? 0.3 : 0;
    }
    if (_type != 5) {
        lcd.drawSprite(_position_x, _position_y, 11, 7, (int *) boss_airship);
    } else {
        lcd.drawSprite(_position_x, _position_y, 19, 30, (int *) boss_plane);
    }
    boss_plane_pixels();
    return check_collision(user_pixels, plane);
}

void BossPlane::fire_rounds(float mx, float my) {
    int random_chance = rand() % 30;
    if (random_chance == 0) {
        tuple<double, double> direction = predict_location(x_pos + 3, y_pos + 3, _position_x + 4, _position_y + 3, mx, my);
        double ex = get<0>(direction);
        double ey = get<1>(direction);
        _boss_bullets.insert(EnemyBullet(_position_x, _position_y, ex, ey, true));
        _boss_bullets.insert(EnemyBullet(_position_x, _position_y, ex + 0.1, ey + 0.1, true));
        _boss_bullets.insert(EnemyBullet(_position_x, _position_y, ex + 0.2, ey + 0.2, true));
        _boss_bullets.insert(EnemyBullet(_position_x, _position_y, ex - 0.1, ey - 0.1, true));
        _boss_bullets.insert(EnemyBullet(_position_x, _position_y, ex - 0.2, ey - 0.2, true));
    }
}

bool BossPlane::update_fire(set<tuple<int, int>> &user_pixels, UserPlane* plane) {
    set<EnemyBullet> temp_set = {};
    for (EnemyBullet bullet : _boss_bullets) {
        auto pos = bullet.get_position();
        int x = get<0>(pos);
        int y = get<1>(pos);
        if (x < 83 || x > 0 || y < 48) {
            if (user_pixels.find(make_tuple(x, y)) != user_pixels.end()) {
                if (plane->plane_hit(50)) {
                    _boss_bullets.clear();
                    return true;
                }
            } else {
                bullet.move();
                temp_set.insert(bullet);
                bullet.draw_bullet();
            }
        }
    }
    _boss_bullets = temp_set;
    return false;
}