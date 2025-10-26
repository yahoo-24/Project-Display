#include "Enemies.h"

EnemyBullet::EnemyBullet(int x, int y, float x_move, float y_move, bool dyn) {
    _x = x;
    _y = y;
    if (dyn) {
        _x_move = x_move; // Move based on input
        _y_move = y_move;
    } else {
        _x_move = 0;
        _y_move = 1.7; // Move straight down
    }
    _dynamic = dyn;
}

void EnemyBullet::draw_bullet() {
    lcd.drawCircle(_x, _y, 1, FILL_BLACK);
}

bool EnemyBullet::operator<(const EnemyBullet& other) const {
    return tie(_x, _y) > tie(other._x, other._y);
}

void EnemyBullet::move() {
    _x += _x_move + wind_x;
    _y += _y_move + wind_y;
}

tuple<int, int> EnemyBullet::get_position() { return make_tuple(_x, _y); }

EnemyPlane::EnemyPlane(float x, float y, char type, int rounds) : _position_x{x}, _position_y{y}, _type{type}, _rounds{rounds} {};

EnemyPlane::EnemyPlane() {}

tuple<float, float> EnemyPlane::get_position() {
    return tuple<float, float> (_position_x, _position_y);
}

bool EnemyPlane::operator<(const EnemyPlane& other) const {
    return _position_x < other._position_x;
}

char EnemyPlane::get_type() { return _type; }

void EnemyPlane::move_plane(float x, float y, set<EnemyBullet> &enemy_fire, float mx, float my) {
    _position_x += x;
    _position_y += y;
    fire_rounds(enemy_fire, mx, my);
}

void EnemyPlane::fire_rounds(set<EnemyBullet> &enemy_fire, float mx, float my) {
    // 1 in 35 chance of firing weapon
    int random_chance = rand() % 35;
    if (random_chance == 0 && _rounds > 0 && _position_y >= -3) {
        // If mx < -99 then predictive targeting is off
        tuple<double, double> direction = (mx <= -99) ? make_tuple(-100.0, -100.0) : predict_location(x_pos + 3, y_pos + 3, _position_x + 4, _position_y + 3, mx, my);
        _rounds -= 1;
        // If direction_x < 99 then just fire straight down else use predicitve targeting
        if (get<0>(direction) <= -99) {
            enemy_fire.insert(EnemyBullet(_position_x + 4, _position_y + 3, 0, 1));
        } else if (abs(get<1>(direction)) > abs(get<0>(direction)) * 0.5) {
            // Add a bit of noise to reduce accuracy. The noise has to be in the center meaning +-x/2 not +x/2
            float noise = (rand() % 10) / 50.0 - 9.0 / 100.0;
            enemy_fire.insert(EnemyBullet(_position_x + 4, _position_y + 3, get<0>(direction) + noise, get<1>(direction) + noise, true));
        }
    }
}

void EnemyPlane::draw_plane() {
    if (_type == 'a') {
        lcd.drawSprite(_position_x, _position_y, 3, 5, (int *)enemy_plane);
    } else if (_type == 'b') {
        lcd.drawSprite(_position_x, _position_y, 4, 7, (int *)larger_enemy_plane);
    } else {
        lcd.drawSprite(_position_x, _position_y, 5, 9, (int *)extra_large_enemy_plane);
    }
}

int EnemyPlane::get_rounds() { return _rounds; }

Formation::Formation() { _unique_id = -999; }

Formation::Formation(short formation_type, float *heatmap, int x) {
    _unique_id = id_counter++;
    x = (x == -99) ? generate_random_position(heatmap) : x; // Randomly generate a position
    x = (x < 72) ? x : 71;
    int y = -4;
    _fixed_path = false;
    _x_move = 0;
    _y_move = 1;
    _formation_type = formation_type;
    switch (formation_type) {
        case 1:
            solo(x, y);
            break;
        case 2:
            vee_formation(x, y);
            break;
        case 3:
            echelon_left(x, y);
            break;
        case 4:
            echelon_right(x, y);
            break;
        case 5:
            double_arrow(x, y);
            break;
        case 6:
            diamond(x, y);
            break;
        case 7:
            vertical_trail(x, y);
            break;
        case 8:
            horizontal_trail(x, y);
            break;
        case 9:
            heavy_left(x, y);
            break;
        case 10:
            heavy_right(x, y);
            break;
        case 11:
            staggered_left(x, y);
            break;
        case 12:
            staggered_right(x, y);
            break;
        case 13:
            zigzag();
            break;
        case 14:
            elliptical();
            break;
        case 15:
            solo_ambush(x);
            break;
        case 16:
            targeting_solo(x, y);
            break;
        case 17:
            wingman(x, y);
            break;
    };
    // All planes start undestroyed
    for (short i = 0; i < _num_of_planes + 1; i++) {
        destroyed[i] = false;
    }
    for (short i = _num_of_planes + 1; i < 6; i++) {
        destroyed[i] = true;
    }
    _remaining = _num_of_planes + 1;
}

Formation Formation::merge(Formation& form1, const Formation& form2, float *heatmap) {
    short total_size = form1._remaining + form2._remaining;
    short type = 1;
    // Planes only re-arrange to these specific formations
    switch (total_size) {
        case 2:
            type = 17; // Wingman formation
            break;
        case 4:
            type = 3; // Echelon left formation
            break;
        case 5:
            type = 2; // Vee formation
            break;
        case 6:
            type = 5; // Double arrow formation
            break;
    }
    Formation new_form = Formation(type, heatmap, get<0>(form1.get_position()));
    new_form._leader = form1._leader; // Set the leader
    for (short i = 0; i < form1._remaining - 1; i++) {
        // Move the followers from the old formation to the new
        new_form._followers[i] = form1._followers[i];
    }
    // The leader of the second formation is now the follower
    new_form._followers[form1._remaining - 1] = form2._leader;
    for (short i = 0; i < form2._remaining - 1; i++) {
        // Move the rest of the followers from the second formation
        new_form._followers[form1._remaining + i] = form2._followers[i];
    }
    new_form._unique_id = id_counter++;
    return new_form;
}

void Formation::vee_formation(int x, int y) {
    _offset[0] = make_tuple(-9, -9);
    _offset[1] = make_tuple(9, -9);
    _offset[2] = make_tuple(-18, -18);
    _offset[3] = make_tuple(18, -18);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 4;
    generate_enemies(x, y);
}

void Formation::echelon_right(int x, int y) {
    _offset[0] = make_tuple(9, -6);
    _offset[1] = make_tuple(18, -15);
    _offset[2] = make_tuple(27, -24);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::echelon_left(int x, int y) {
    _offset[0] = make_tuple(-9, -6);
    _offset[1] = make_tuple(-18, -15);
    _offset[2] = make_tuple(-27, -24);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::double_arrow(int x, int y) {
    _offset[0] = make_tuple(9, -6);
    _offset[1] = make_tuple(-9, -6);
    _offset[2] = make_tuple(0, -15);
    _offset[3] = make_tuple(9, -21);
    _offset[4] = make_tuple(-9, -21);
    _num_of_planes = 5;
    generate_enemies(x, y);
}

void Formation::diamond(int x, int y) {
    _offset[0] = make_tuple(9, -6);
    _offset[1] = make_tuple(-9, -6);
    _offset[2] = make_tuple(0, -15);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::vertical_trail(int x, int y) {
    _offset[0] = make_tuple(0, -10);
    _offset[1] = make_tuple(0, -20);
    _offset[2] = make_tuple(0, -30);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::horizontal_trail(int x, int y) {
    _offset[0] = make_tuple(-10, 0);
    _offset[1] = make_tuple(-20, 0);
    _offset[2] = make_tuple(-30, 0);
    _offset[3] = make_tuple(-40, 0);
    _offset[4] = make_tuple(-50, 0);
    _num_of_planes = 5;
    generate_enemies(x, y);
}

void Formation::heavy_left(int x, int y) {
    _offset[0] = make_tuple(8, -3);
    _offset[1] = make_tuple(-10, -7);
    _offset[2] = make_tuple(-18, -10);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::heavy_right(int x, int y) {
    _offset[0] = make_tuple(-8, -3);
    _offset[1] = make_tuple(10, -7);
    _offset[2] = make_tuple(18, -10);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::staggered_left(int x, int y) {
    _offset[0] = make_tuple(-9, -6);
    _offset[1] = make_tuple(-12, 0);
    _offset[2] = make_tuple(-21, -6);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::staggered_right(int x, int y) {
    _offset[0] = make_tuple(9, -6);
    _offset[1] = make_tuple(12, 0);
    _offset[2] = make_tuple(21, -6);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(x, y);
}

void Formation::solo(int x, int y) {
    _offset[0] = make_tuple(0 ,0);
    _offset[1] = make_tuple(0, 0);
    _offset[2] = make_tuple(0, 0);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 0;
    generate_enemies(x, y);
}

void Formation::zigzag() {
    _fixed_path = true;
    _offset[0] = make_tuple(-8, -6);
    _offset[1] = make_tuple(-16, -12);
    _offset[2] = make_tuple(-24, -18);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 3;
    generate_enemies(0, 7);
}

void Formation::elliptical() {
    _fixed_path = true;
    _num_of_planes = 2;
    _x_move = 1;
    _offset[0] = make_tuple(-7, 0);
    _offset[1] = make_tuple(-14, 0);
    _offset[2] = make_tuple(0, 0);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    generate_enemies(0, 3 + rand() % 8);
}

void Formation::solo_ambush(int x) {
    _fixed_path = true;
    _offset[0] = make_tuple(0, 0);
    _offset[1] = make_tuple(0, 0);
    _offset[2] = make_tuple(0, 0);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 0;
    generate_enemies(x, 48);
}

void Formation::targeting_solo(int x, int y) {
    _fixed_path = true;
    _offset[0] = make_tuple(0, 0);
    _offset[1] = make_tuple(0, 0);
    _offset[2] = make_tuple(0, 0);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 0;
    generate_enemies(x, y);
}

void Formation::wingman(int x, int y) {
    _offset[0] = make_tuple(-12, -12);
    _offset[1] = make_tuple(0, 0);
    _offset[2] = make_tuple(0, 0);
    _offset[3] = make_tuple(0, 0);
    _offset[4] = make_tuple(0, 0);
    _num_of_planes = 1;
    generate_enemies(x, y);
}

void Formation::zigzag_move(int x, int y) {
    _y_move = 0.5;
    if (y < 20 || y > 30) {
        _x_move = 1;
    } else {
        _x_move = -1;
    }
}

void Formation::elliptical_move(float x, float y, short i) {
    // Moves along a fixed path
    if ((x >= 64 && y < 30) || (y >= 12 && y < 40 && x < 12)) {
        _x_move = sin(_theta[i] * 3.14159 / 180);
        _y_move = abs(cos(_theta[i] * 3.14159 / 180));
        _theta[i] -= 10;
    } else if (y >= 24 && x < x_pos + 4 && x > x_pos - 4) {
        _x_move = 0;
        _y_move = 1.5;
    } else {
        if (y < 12 || y >= 24) {
            _x_move = 1;
            _y_move = 0;
        } else if (y >= 12 && y < 24) {
            _x_move = -1;
            _y_move = 0;
        }
    }
}

void Formation::ambush_move(int y) {
    if (y < 38) {
        _y_move = -0.5;
    } else {
        _y_move = -1.3;
    }
}

void Formation::targeting_move(int x, int y) {
    if (y < 20) {
        _y_move = 1;
    } else {
        int diff = x_pos - x;
        _x_move = 0.05 * diff;
        _x_move = (_x_move > 1) ? 1 : (_x_move < -1) ? -1 : _x_move;
    }
}

void Formation::generate_enemies(int x, int y) {
    EnemyPlane enemy = EnemyPlane(x, y, 'c');
    _leader = enemy;
    for (short i = 0; i < _num_of_planes; i++) {
        enemy = EnemyPlane(x + get<0>(_offset[i]), y + get<1>(_offset[i]), 'c');
        _followers[i] = enemy;
    }
}

bool Formation::move_planes(bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char &special, set<EnemyBullet> &enemy_fire, float mx, float my, FighterPlane &support_1, FighterPlane &support_2) { 
    bool result = check_collisions(_leader, user_hit, user_plane, plane_pixels, bullet_set, special, support_1, support_2);
    destroyed[0] = result;
    if (destroyed[0]) {
        // Delete the formation if the leader is dead: only happens when no planes left.
        _remaining--;
        if (_remaining == 0) {
            return true;
        }
        // If there are other planes then the next one is the leader.
        _leader = _followers[0];
        // Re-arrange the arrays by shifting the elements to the left.
        rotate(_followers, _followers + 1, _followers + _num_of_planes);
        rotate(destroyed, destroyed + 1, destroyed + _num_of_planes + 1);
    }
    draw_plane(_leader);
    // Check position after collision check in case there is a new leader.
    tuple<float, float> pos = _leader.get_position(); // Position of the leader
    if (_formation_type == 13) {
        zigzag_move(get<0>(pos), get<1>(pos));
    } else if (_formation_type == 14) {
        elliptical_move(get<0>(pos), get<1>(pos), 0);
    } else if (_formation_type == 15) {
        ambush_move(get<1>(pos));
    } else if (_formation_type == 16) {
        targeting_move(get<0>(pos), get<1>(pos));
    }
    _leader.move_plane(_x_move, _y_move, enemy_fire, mx, my);
    return move_followers(pos, user_hit, user_plane, plane_pixels, bullet_set, special, enemy_fire, mx, my, support_1, support_2) && (get<0>(pos) > 79 || get<0>(pos) < 0 || get<1>(pos) > 52);
}

bool Formation::move_followers(tuple<float, float> pos, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char &special, set<EnemyBullet> &enemy_fire, float mx, float my, FighterPlane &support_1, FighterPlane &support_2) {
    bool outside = true;
    float move_follower_x = 0;
    float move_follower_y = 0;
    short diff; // Difference in positions to check if it aligns with offset
    tuple<float, float> pos2; // Position of the follower
    for (int i = 0; i < _num_of_planes; i++) {
        if (destroyed[i+1]) {
            break;
        }
        bool result = check_collisions(_followers[i], user_hit, user_plane, plane_pixels, bullet_set, special, support_1, support_2);
        destroyed[i+1] = result;
        if (result) {
            // If collision re-arrange the formation.
            _remaining--;
            if (i == _num_of_planes - 1) {
                break;
            }
            // The arrays are being re-arranged.
            rotate(_followers + i, _followers + i + 1, _followers + _num_of_planes);
            rotate(destroyed + i + 1, destroyed + i + 2, destroyed + _num_of_planes + 1);
        } else {
            draw_plane(_followers[i]);
        }

        // Check if path is fixed to re-align formation.
        // Fixed paths do not need re-aligning
        pos2 = _followers[i].get_position();
        if (!_fixed_path) {
            // Re-align the plane with the offset value in the x-direction
            // Add checks to verify that it does not go out of the screen.
            if (get<0>(pos) + get<0>(_offset[i]) > 72) {
                get<0>(_offset[i]) = 72 - get<0>(pos);
            } else if (get<0>(pos) + get<0>(_offset[i]) < 0) {
                get<0>(_offset[i]) = get<0>(pos);
            }
            diff = get<0>(pos2) - get<0>(pos);
            if (diff > get<0>(_offset[i])) {
                // Adding more movement to realign it faster to its formation
                move_follower_x = -0.8;
            } else if (diff < get<0>(_offset[i])) {
                move_follower_x = 0.8;
            } else {
                move_follower_x = 0;
            }
            // Re-align the plane with the offset value in the y-direction
            diff = get<1>(pos2) - get<1>(pos);
            if (diff > get<1>(_offset[i])) {
                // Adding more movement to realign it faster to its formation
                // y-direction movement is less than x to prevent near complete halt.
                move_follower_y = -0.5;
            } else if (diff < get<1>(_offset[i])) {
                move_follower_y = 0.5;
            } else {
                move_follower_y = 0;
            }
        } else {
            if (_formation_type == 14) {
                elliptical_move(get<0>(pos2), get<1>(pos2), i + 1);
            } else {
                zigzag_move(get<0>(pos2), get<1>(pos2));
            }
        }
        _followers[i].move_plane(_x_move + move_follower_x, _y_move + move_follower_y, enemy_fire, mx, my);

        if (get<0>(pos2) < 79 && get<0>(pos2) > 0 && get<1>(pos2) < 52) {
            // As long as 1 plane is within the screen limits, false is returned
            outside = false;
        }
    }
    return outside;
}

bool Formation::operator<(const Formation& other) const {
    return _unique_id > other._unique_id;
}

EnemyPlane Formation::get_leader() { return _leader; }

EnemyPlane* Formation::get_followers() { return _followers; }

short Formation::get_num_planes() { return _num_of_planes; }

short Formation::get_formation_type() { return _formation_type; }

float Formation::get_x_move() { return _x_move; }

float Formation::get_y_move() { return _y_move; }

tuple<float, float> Formation::get_position() { return _leader.get_position(); }

short Formation::get_remaining() { return _remaining; }

void Formation::draw_plane(EnemyPlane &plane) {
    if (_formation_type == 14) {
        if (abs(_x_move) > abs(_y_move)) {
            tuple<int, int> pos = plane.get_position();
            if (_x_move > 0) {
                lcd.drawSprite(get<0>(pos), get<1>(pos), 9, 5, (int *)extra_large_enemy_plane_right);
            } else {
                lcd.drawSprite(get<0>(pos), get<1>(pos), 9, 5, (int *)extra_large_enemy_plane_left);
            }
        } else {
            plane.draw_plane();
        }
    } else if (_formation_type == 15) {
        tuple<int, int> pos = plane.get_position();
        lcd.drawSprite(get<0>(pos), get<1>(pos), 5, 9, (int *)extra_large_enemy_plane_back);
    } else {
        plane.draw_plane();
    }
}

long Formation::id_counter = 0;

void heatmap_sum(float *heatmap, int* cum_sum) {
    int total = 0;
    for (int i = 0; i < 4; i++) {
        total += (int) heatmap[i];
        cum_sum[i] = total;
    }
}

int generate_random_position(float *heatmap) {
    // Generates a position with the probability affected by the heatmap
    int cum_sum[4];
    heatmap_sum(heatmap, cum_sum);
    float sum = cum_sum[3];
    int add = 0;
    int section = rand() % 6720;
    if (section < (cum_sum[0] / sum) * 6720) {
        add = 0;
    } else if (section > (cum_sum[0] / sum) * 6720 && section < (cum_sum[1] / sum) * 6720) {
        add = 20;
    } else if (section > (cum_sum[1] / sum) * 6720 && section < (cum_sum[2] / sum) * 6720) {
        add = 40;
    } else {
        add = 60;
    }
    return rand() % 20 + add;
}

bool bullet_hit(float x, int y, float xb, int yb, int side) {
    // side is checked as bullets can move right or left and side is set to that accordingly
    bool cond1 = ((xb >= x + side) && (xb <= x + 9 + side) && (yb == y || yb == y - 1));
    bool cond2 = ((xb >= x + 1 + side) && (xb <= x + 8 + side) && (yb == y + 1));
    bool cond3 = ((xb >= x + 2 + side) && (xb <= x + 7 + side) && (yb == y + 2));
    bool cond4 = ((xb >= x + 3 + side) && (xb <= x + 6 + side) && (yb == y + 3));
    bool cond5 = ((xb >= x + 4 + side) && (xb <= x + 5 + side) && (yb == y + 4));
    return (cond1 || cond2 || cond3 || cond4 || cond5);
}

bool find_bullet_helper(map<char, set<tuple<float, float>>> &bullet_set, int x, int y, char type, char special) {
    int side = 0;
    bool hit = false;
    if (type == 'R') {
        side = 1;
    } else if (type == 'L') {
        side = -1;
    }
    for (tuple<float, float> bullet : bullet_set[type]) {
        if(bullet_hit(x, y, get<0>(bullet), get<1>(bullet), side)) {
            if (special != 'A') {
                bullet_set[type].erase(bullet);
            }
            hit = true;
            return true;;
        }
    }
    return false;
}

bool find_bullet(map<char, set<tuple<float, float>>> &bullet_set, int x, int y, char special) {
    return find_bullet_helper(bullet_set, x, y, 'S', special) || find_bullet_helper(bullet_set, x, y, 'L', special) || find_bullet_helper(bullet_set, x, y, 'R', special);
}

bool check_collisions(EnemyPlane &element, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char special, FighterPlane &support_1, FighterPlane &support_2) {
    int size_x, size_y;
    bool hit = false;
    tuple<float, float> position = element.get_position();
    float x = get<0>(position);
    float y = get<1>(position);
    // Loop through the enemy pixels and check for collision
    size_x = 9;
    size_y = 5;
    if (find_bullet(bullet_set, x, y, special)) {
        // If hit by a bullet then delete the bullet and raise the hit flag
        hit = true;
    }
    // Check for collision with the user
    for (int i = 0; i < size_y; i++) {
        for (int j = i; j < size_x-i; j++) {
            if (plane_pixels.find(make_tuple(x + j, y + i)) != plane_pixels.end()) {
                if (user_plane->get_power_up() != "Invincible") {
                    // Check that the user is not invincible
                    user_hit = true;
                    user_plane->lost_life();
                }
                hit = true;
            }
            // Check if it hits the support as well
            if (support_1.check_collision(make_tuple(x + j, y + i), false) || support_2.check_collision(make_tuple(x + j, y + i), false)) {
                hit = true;
            }
            if (hit) {
                // Draw an explosion and stop loop.
                if (y > 0) { lcd.drawCircle(x + 3, y + 1, 4, FILL_BLACK); }
                user_plane->score_points(1);
                return true;
            }
        }
    }
    return hit;
}

tuple<double, double> predict_location(float pos_x, float pos_y, float e_pos_x, float e_pos_y, float mx, float my) {
    // See the calculations for details on what is happening below.
    float delta_pos_x = pos_x - e_pos_x;
    float delta_pos_y = pos_y - e_pos_y;
    float delta_pos = pow(delta_pos_x, 2) + pow(delta_pos_y, 2);
    float m = pow(mx, 2) + pow(my, 2);
    float b = 2*mx*delta_pos_x + 2*my*delta_pos_y;
    double temp1 = pow(b, 2) + 4*delta_pos*(1.44-m);
    double temp2 = pow(b, 2) + 4*delta_pos*(4-m);
    double t1 = (temp1 >= 0) ? b - sqrt(temp1) : -10000;
    double t2 = (temp1 >= 0) ? b + sqrt(temp1) : -10000;
    double t3 = (temp2 >= 0) ? b - sqrt(temp2) : -10000;
    double t4 = (temp2 >= 0) ? b + sqrt(temp2) : -10000;
    t1 /= 2*(1.44-m);
    t2 /= 2*(1.44-m);
    t3 /= 2*(4-m);
    t4 /= 2*(4-m);
    int time = -1;
    if (t4 < t2 && (int) t4 != (int) t2 && t4 > 0) {
        time = t4 + 1;
    } else if (t4 < t1 && (int) t4 != (int) t1) {
        time = t1 + 1;
    }
    if (time == -1) {
        return make_tuple(-100, -100);
    }
    double ex = (mx*time + delta_pos_x) / time;
    double ey = (my*time + delta_pos_y) / time;
    return make_tuple(ex, ey);
}

void classify_formation(map<short, set<Formation>> &formation_merge, Formation &formation) {
    // classify them based on their screen location Left, Center, Right
    tuple<int, int> position = formation.get_position();
    short type = formation.get_formation_type();
    if (type >= 13 && type <= 16) { return; }
    if (get<1>(position) < 25) {
        int x = get<0>(position);
        if (x < 24) {
            formation_merge[0].insert(formation);
        } else if (x < 48) {
            formation_merge[1].insert(formation);
        } else {
            formation_merge[2].insert(formation);
        }
    }
}

void formation_merger(set<Formation> &formations_to_merge, set<Formation> &formations, float* heatmap) {
    set<Formation> formations_to_delete = {};
    set<Formation> merged_formations = {};
    if (formations_to_merge.size() > 1) {
        auto it_front = formations_to_merge.begin();
        auto end = prev(formations_to_merge.end()); // The iterator for the last item in the list
        auto it_back = end;
        Formation form2;
        // prev(end()) is used rather than end() because the last object cannot merge with anything else
        // since there is nothing that comes after it.
        while (it_front != end) {
            Formation form1 = *it_front;
            if (formations_to_delete.find(form1) != formations_to_delete.end()) {
                // Move the iterator to avoid infinite loop and move to the next object in the set.
                it_front++;
                continue;
            }
            it_back = prev(formations_to_merge.end());
            while (it_front != it_back) {
                form2 = *it_back;
                short size = form1.get_remaining() + form2.get_remaining();
                if (size < 7 && formations_to_delete.find(form2) == formations_to_delete.end()) {
                    if (rand() % 3 == 0) {
                        // There is a 1/3 chance of merging
                        merged_formations.insert(Formation::merge(form1, form2, heatmap));
                        // To delete later to avoid duplicates
                        formations_to_delete.insert(form1);
                        formations_to_delete.insert(form2);
                        // Break so it only merges once
                        break;
                    }
                }
                it_back--; // Iterator moves backwards towards the front iterator
            }
            it_front++;
        }
        // Delete the formations that have been merged.
        for (Formation formation : formations_to_delete) {
            formations.erase(formation);
        }
        // Add the newly made formations.
        for (Formation formation : merged_formations) {
            formations.insert(formation);
        }
    }
}