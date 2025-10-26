#include "Funcs.h"

// The user chooses the plane  using the plane menu
UserPlane* get_choice(int choice) {
    switch(choice) {
        case 1:
            return new AttackingPlane();
            break;
        case 2:
            return new FighterPlane();
            break;
        case 3:
            return new Bomber();
            break;
        default:
            return new AttackingPlane();
    }
}

// Resets the index so we get a circular array
void UserTracker::update_index_x() {
    _index_x = (_index_x == 29) ? 0 : _index_x += 1;
}

void UserTracker::update_index_y() {
    _index_y = (_index_y == 29) ? 0 : _index_y += 1;
}

UserTracker::UserTracker() {}

// Adds the new value to the queue and updates the sum of the array
void UserTracker::update_x(float val_x) {
    _sum_x -= _displacements_x[_index_x];
    _displacements_x[_index_x] = val_x;
    _sum_x += val_x;
    update_index_x();
}

void UserTracker::update_y(float val_y) {
    _sum_y -= _displacements_y[_index_y];
    _displacements_y[_index_y] = val_y;
    _sum_y += val_y;
    update_index_y();
}

float UserTracker::get_average_x() {
    return _sum_x / 30;
}

float UserTracker::get_average_y() {
    return _sum_y / 30;
}

// Update the user location based on the joystick input
void check_movement(Vector2D coord, float sf, UserTracker &tracker) {
    float temp_x = x_pos + wind_x / 2 + sf * coord.x;
    if (temp_x < 73 && temp_x > 0 && abs(coord.x) > 0.1) {
        x_pos += sf * coord.x + wind_x / 2; // Wind is added at half its magnitude
        tracker.update_x(sf * coord.x);
    } else {
        tracker.update_x(0);
    }

    float temp_y = y_pos - sf * coord.y + wind_y;
    if (temp_y < 41 && temp_y > 0 && abs(coord.y) > 0.1) {
        y_pos += wind_y - sf * coord.y;
        tracker.update_y(sf * coord.y);
    } else {
        tracker.update_y(0);
    }
}

set<tuple<int, int>> update_plane_pixels(UserPlane* plane) {
    // Setting up a tuple for all the user plane's pixel positions.
    set<tuple<int, int>> pixels = {};
    for (int i = 0; i < plane->get_height(); i++) {
        for (int j = 0; j < plane->get_width(); j++) {
            if (*(plane->sprite + i + j) == 1) {
                pixels.insert(make_tuple(x_pos + j, y_pos + i));
            }
        }
    }
    return pixels;
}

void joystick_button_isr() {
    g_buttonA_flag = 1;
}

void special_button_isr() {
    g_special_flag = 1;
}

void fire_rate_isr() {
    g_fire_timer = 1;
}

void special_ability_isr() {
    g_special_ability_timer = 1;
}

void pause_button_isr() {
    g_pause_flag = 1;
}

void round_end_isr() {
    g_round_end = 1;
}

void survival_isr() {
    g_survival = 1;
}

bool check_if_user_fired_weapon(UserPlane *user_plane, char &special, float &counter, map<char, set<tuple<float, float>>> &bullet_set_float, FighterPlane &support_1, FighterPlane &support_2) {
    // Check if the user fired the weapon and add the bullet to the screen
    if (g_buttonA_flag) {
        g_buttonA_flag = 0;
        if (g_fire_timer) {
            user_plane->fire_weapon(bullet_set_float);
            support_1.support_fire(bullet_set_float);
            support_2.support_fire(bullet_set_float);
            g_fire_timer = 0;
            fire_rate.detach();
            fire_rate.attach(&fire_rate_isr, user_plane->get_fire_rate());
            if (special == 'A') {
                counter++;
                if (counter >= 15) {
                    counter = 0;
                    special = 'x';
                }
            }
            return true;
        }
    }
    return false;
}

void generate_enemy_planes(int points, set<Formation> &formations, int* min_points, int factor, float* heatmap) {
    int enemy_gen = 0;
    int type = 0;
    // As the user gets more points, the probability of a plane spawning is more likely.
    if (points / 7 >= factor - 20) {
        enemy_gen = rand() % 20;
    } else {
        enemy_gen = rand() % (factor - (points / 14));
    }
    if (enemy_gen == 1) {
        // The user has to exceed the minimum points to unlock other plane formations
        for (short i = 16; i >= 0; i--) {
            if (points >= min_points[i]) {
                type = rand() % (i + 1) + 1;
                break;
            }
        }
        formations.insert(Formation(type, heatmap));
    }
}

bool generate_booster(int probability) {
    return (rand() % probability == 1) ? true : false;
}

bool update_boosters(PowerUps* booster_arr, set<tuple<int, int>> pixels, UserPlane *user) {
    bool booster_collected = false;
    for (short i = 0; i < 8; i++) {
        if (booster_arr[i].active) {
            // Check if the booster is out of bounds or collected
            if (booster_arr[i].booster.move(pixels, user, booster_collected)) {
                booster_arr[i].active = false;
            } else {
                booster_arr[i].booster.draw_sprite();
            }
        } else {
            // Generate the booster. A check has to be made on the extra life booster so the user does not get more than 3 lives
            if (booster_arr[i].booster.get_type() != "Life" ||
            (booster_arr[i].booster.get_type() == "Life" && user->get_lives() < 3)) {
                booster_arr[i].active = generate_booster(booster_arr[i].probability);
            }
        }
    }
    return booster_collected; // Used for the buzzer
}

void update_user_bullets_helper(map<char, set<tuple<float, float>>> &bullet_set_float, char type) {
    set<tuple<float, float>> bullet_set_temp = {};
    for (tuple<float, float> element : bullet_set_float[type]) {
        float x = get<0>(element);
        float y = get<1>(element);
        if (type == 'L') {
            x -= 1; // Left moving bullets
        } else if (type == 'R') {
            x += 1; // Right moving bullets
        }
        x += wind_x; // Add wind to the bullet
        y += -1 + wind_y;
        if (y > -1 && x > -1 && x < 80) {
            bullet_set_temp.insert(make_tuple(x, y));
            if (type == 'L') {
                lcd.drawSprite(x, y, 2, 2, (int *)bullet_left);
            } else if (type == 'R') {
                lcd.drawSprite(x, y, 2, 2, (int *)bullet_right);
            } else {
                lcd.drawSprite(x, y, 2, 1, (int *)bullet);
            }
        }
    }
    bullet_set_float[type] = bullet_set_temp;
}

void update_user_bullets(map<char, set<tuple<float, float>>> &bullet_set_float) {
    // Loop through all the bullets and move them one step forward and check they are in range
    update_user_bullets_helper(bullet_set_float, 'S');
    update_user_bullets_helper(bullet_set_float, 'R');
    update_user_bullets_helper(bullet_set_float, 'L');
}

void update_enemy_fire(set<EnemyBullet> &enemy_fire, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, FighterPlane &support_1, FighterPlane &support_2) {
    set<EnemyBullet> enemy_fire_temp = {};
    for (EnemyBullet round : enemy_fire) {
        if (user_hit) {
            break; // Stop checking if the user is dead
        }
        int x = get<0>(round.get_position());
        int y = get<1>(round.get_position());
        // Look if the bullet has reached any
        if (plane_pixels.find(make_tuple(x, y)) != plane_pixels.end() || 
        plane_pixels.find(make_tuple(x + 1, y)) != plane_pixels.end() || plane_pixels.find(make_tuple(x - 1, y))!= plane_pixels.end()) {
            // Plane not affected in invincibility
            // With deflect the bullet goes into the user's bullet array
            if (user_plane->get_power_up() != "Invincible") {
                if (user_plane->get_power_up() == "Deflect") {
                    bullet_set['S'].insert(make_tuple(x, y));
                } else {
                    user_hit = user_plane->plane_hit(20);
                }
            }
        } else if (!support_1.check_collision(make_tuple(x, y), true) && !support_2.check_collision(make_tuple(x, y), true)) {
            if (y < 50 && y >= 0) {
                round.move();
                round.draw_bullet();
                enemy_fire_temp.insert(round);
            }
        }
    }
    enemy_fire = enemy_fire_temp;
}

void check_user_hit(bool &user_hit, set<Formation> &formations, map<char, set<tuple<float, float>>> &bullet_set, set<EnemyBullet> &enemy_fire) {
    if (user_hit) {
        lcd.drawCircle(x_pos + 3, y_pos + 3, 8, FILL_BLACK);
        lcd.refresh(); // need to refresh the screen to get the message to appear

        // Play the buzzer
        buzzer.period_us(1000000.0 / NOTE_G5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(50ms);
        buzzer.period_us(1000000.0 / NOTE_E5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(50ms);
        buzzer.period_us(1000000.0 / NOTE_C5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(50ms);
        buzzer.period_us(1000000.0 / NOTE_G4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(70ms);
        buzzer.period_us(1000000.0 / NOTE_E4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(80ms);
        buzzer.period_us(1000000.0 / NOTE_C4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        ThisThread::sleep_for(100ms);
        buzzer.write(0);

        // Reset all the enemies and bullets
        user_hit = false;
        enemy_fire.clear();
        formations.clear();
        bullet_set['S'].clear();
        bullet_set['L'].clear();
        bullet_set['R'].clear();
        ThisThread::sleep_for(2000ms);
    } else {
        lcd.refresh();
    }
}

void game_over_screen(UserPlane *user_plane, int score, bool success) {
    lcd.clear();
    lcd.printString("MISSION REPORT", 0, 0);
    ThisThread::sleep_for(1s);
    lcd.printString("OP: SKYSTORM", 0, 1);
    ThisThread::sleep_for(1s);
    if (success) {
        lcd.printString("OP SUCCESS", 0, 2);
    } else {
        lcd.printString("MISSION FAIL", 0, 2);
    }
    ThisThread::sleep_for(1s);
    char buffer[14] = {0};
    sprintf(buffer, "KILLS: %d", user_plane->get_points() + score);
    lcd.printString(buffer, 0, 3);
    ThisThread::sleep_for(1s);
    if (user_plane->get_lives() > 0) {
        lcd.printString("STATUS: ACTIVE", 0, 4);
    } else {
        lcd.printString("STATUS: KIA", 0, 4);
    }
    lcd.refresh();
}

int atomic_bomb_helper_function(Formation &formation) {
    // Loops through the enemies set and draws an explosion in the enemy's position.
    // A check is made when drawing the circle to prevent MCU freeze.
    int count = 0;
    tuple<int, int> position = formation.get_leader().get_position();
    lcd.drawCircle(get<0>(position) + 4, get<1>(position) + 1, 4, FILL_BLACK);
    EnemyPlane* enemies = formation.get_followers();
    for (short i = 0; i < formation.get_num_planes(); i++) {
        count++;
        tuple<int, int> position = enemies[i].get_position();
        int x = get<0>(position);
        int y = get<1>(position);
        if (y > 3) {
            lcd.drawCircle(x + 4, y + 1, 4, FILL_BLACK);
        } else if (y >= 0) {
            lcd.drawCircle(x + 4, y + 1, y + 1, FILL_BLACK);
        }
    }
    return count;
}

int atomic_bomb(set<Formation> &formations, set<EnemyBullet> &enemy_fire) {
    // This is the bomber's ability. Clears the entire screen of enemies.
    int total = 0;
    enemy_fire.clear();
    for (Formation formation : formations) {
        total += atomic_bomb_helper_function(formation);
    }
    formations.clear();
    return total;
}

void fighter_plane_support(float &counter, char &type, map<char, set<tuple<float, float>>> &bullet_set) {
    int temp = 0;
    if (counter > 52) {
        counter = 0;
        type = 'x';
    } else {
        // Move the planes upwards
        counter += 0.5;
        temp = 52 - counter; // They start from 52 which is the bottom
        lcd.drawSprite(6, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(15, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(24, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(33, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(42, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(51, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(60, temp, 6, 5, (int *) fighter_plane);
        lcd.drawSprite(69, temp, 6, 5, (int *) fighter_plane);
        if ((int) temp % 5 == 0) {
            bullet_set['S'].insert(make_tuple(8, temp));
            bullet_set['S'].insert(make_tuple(17, temp));
            bullet_set['S'].insert(make_tuple(26, temp));
            bullet_set['S'].insert(make_tuple(35, temp));
            bullet_set['S'].insert(make_tuple(44, temp));
            bullet_set['S'].insert(make_tuple(53, temp));
            bullet_set['S'].insert(make_tuple(62, temp));
            bullet_set['S'].insert(make_tuple(71, temp));
        }
    }
}

void adjust_heatmap(float *heatmap) {
    // Add value to the area the user is now
    if (x_pos < 20) {
        heatmap[0] += 0.005;
    } else if (x_pos < 40) {
        heatmap[1] += 0.005;
    } else if (x_pos < 60) {
        heatmap[2] += 0.005;
    } else {
        heatmap[3] += 0.005;
    }
}

int plane_menu() {
    struct State {
        int choice;
        int hitpoints;
        float speed_factor;
        int next_state[3];
    };

    State plane_fsm[3] = {
        {1, 100, 1, {2, 0, 1}},
        {2, 60, 1.3, {0, 1, 2}},
        {3, 200, 0.7, {1, 2, 0}}
    };

    int direction;
    int state = 0;
    int hp;
    int speed;
    while(!g_buttonA_flag) {
        // Determine if the user swiped right or left or none.
        Vector2D coord = joystick.get_mapped_coord(); // See if the user moved right or left
        if (coord.x > 0.5) {
            direction = 2;
        } else if (coord.x < -0.5) {
            direction = 0;
        } else {
            direction = 1;
        }

        // Determine the next state
        state = plane_fsm[state].next_state[direction];
        // Factor the speed and hp for display
        hp = plane_fsm[state].hitpoints / 5;
        speed = plane_fsm[state].speed_factor * 20;

        // Draw the Plane menu
        lcd.clear();
        switch (plane_fsm[state].choice) {
            case 1:
                lcd.drawSprite(39, 3, 7, 7, (int *)attack_plane);
                break;
            case 2:
                lcd.drawSprite(39, 3, 6, 5, (int *)fighter_plane);
                break;
            case 3:
                lcd.drawSprite(39, 3, 8, 9, (int *)bomber_plane);
                break;
            default:
                lcd.drawSprite(39, 3, 7, 7, (int *)attack_plane);
        }
        lcd. printString("HP:", 1, 2);
        lcd.drawRect(24, 18, hp, 6, FILL_BLACK);
        lcd.drawRect(24 + hp, 18, 40 - hp, 6, FILL_TRANSPARENT);
        lcd.printString("m/s", 1, 4);
        lcd.drawRect(24, 32, speed, 6, FILL_BLACK);
        lcd.drawRect(24 + speed, 32, 26 - speed, 6, FILL_TRANSPARENT);
        lcd.printString("    SELECT", 0, 5);
        lcd.refresh();
        ThisThread::sleep_for(200ms);
    }
    g_buttonA_flag = 0;
    return plane_fsm[state].choice;
}

void decay_heatmap(float* heatmap) {
    // Reduce all the values by half but keep them above or equal to 1
    for (int i = 0; i < 4; i++) {
        heatmap[i] /= 2;
        if (heatmap[i] < 1) {
            heatmap[i] = 1;
        }
    }
}

Cloud::Cloud() {
    _x = rand() % 59;
    _y = 0;
}

void Cloud::draw_cloud() {
    // Manually drawing the cloud instead of using a sprite.
    // This ignores the white spaces.
    // It takes less operations to draw it this way.
    for (short i = 0; i < 13; i++) {
        lcd.setPixel(_x + i, _y - 3);
        lcd.setPixel(_x + i, _y - 4);
        lcd.setPixel(_x + i, _y - 5);
        lcd.setPixel(_x + i, _y - 6);
    }
    for (short i = 1; i < 12; i++) {
        lcd.setPixel(_x + i, _y - 2);
    }
    for (short i = 3; i < 11; i++) {
        lcd.setPixel(_x + i, _y - 1);
        lcd.setPixel(_x + i, _y - 7);
    }
    lcd.setPixel(_x + 2, _y - 7);
    lcd.setPixel(_x + 5, _y);
    lcd.setPixel(_x + 6, _y);
    lcd.setPixel(_x + 7, _y);
    for (short i = 4; i < 8; i++) {
        lcd.setPixel(_x + i, _y - 10);
        lcd.setPixel(_x + i, _y - 9);
        lcd.setPixel(_x + i, _y - 8);
    }
    lcd.setPixel(_x + 2, _y - 8);
    lcd.setPixel(_x + 3, _y - 8);
    lcd.setPixel(_x + 3, _y - 9);
    lcd.setPixel(_x + 8, _y - 8);
    lcd.setPixel(_x + 8, _y - 9);
    /*
    Pass the sprite as an argument.
    for (int i = 0; i < sprite_size_x; i++) {
        for (int j = 0; j < sprite_size_y; j++) {
            if (sprite[i][j] == 1) {
                lcd.setPixel(_x + j, _y + i);
            }
        }
    }
    */
}

bool Cloud::move_cloud() {
    _y += 0.4 + wind_y;
    _x += wind_x;
    // Is it out of range
    if (_y > 59 || _x > 80 || _x < -13) {
        return true;
    }
    return false;
}

bool Cloud::operator<(const Cloud& other) const {
    return tie(_x, _y) > tie(other._x, other._y);
}

void generate_cloud(set<Cloud> &clouds, int probability_factor) {
    int chance = rand() % probability_factor;
    if (chance == 0) { clouds.insert(Cloud()); }
}

void move_clouds(set<Cloud> &clouds) {
    set<Cloud> temp_clouds = {};
    for (Cloud cloud : clouds) {
        // Check that the cloud is still in range after moving it
        if (!cloud.move_cloud()) {
            temp_clouds.insert(cloud);
            cloud.draw_cloud();
        }
    }
    clouds = temp_clouds;
}

void set_brightness(float &brightness) {
    ThisThread::sleep_for(200ms);
    joystick_button.enable_irq();
    while (!g_buttonA_flag) {
        // Update brightness based on joystick input
        lcd.clear();
        lcd.printString("Brightness", 14, 0);
        lcd.drawRect(17, 20, 50, 10, FILL_TRANSPARENT);
        lcd.drawRect(17, 20, 50 * brightness, 10, FILL_BLACK);
        Vector2D coord = joystick.get_mapped_coord();
        if (coord.x > 0.2) {
            brightness += (brightness >= 0.94) ? 0 : 0.05;
            lcd.setBrightness(brightness);
        } else if (coord.x < -0.2) {
            brightness -= (brightness <= 0.06) ? 0 : 0.05;
            lcd.setBrightness(brightness);
        }
        lcd.refresh();
        ThisThread::sleep_for(150ms);
    }
    joystick_button.disable_irq();
    g_buttonA_flag = 0;
}

void set_contrast(float &contrast) {
    ThisThread::sleep_for(200ms);
    joystick_button.enable_irq();
    while (!g_buttonA_flag) {
        lcd.clear();
        lcd.printString("Contrast", 14, 0);
        lcd.drawRect(17, 20, 50, 10, FILL_TRANSPARENT);
        lcd.drawRect(17, 20, 50 * contrast, 10, FILL_BLACK);
        Vector2D coord = joystick.get_mapped_coord();
        if (coord.x > 0.2) {
            contrast += (contrast >= 0.94) ? 0 : 0.05;
            lcd.setContrast(contrast);
        } else if (coord.x < -0.2) {
            contrast -= (contrast <= 0.06) ? 0 : 0.05;
            lcd.setContrast(contrast);
        }
        lcd.refresh();
        ThisThread::sleep_for(150ms);
    }
    joystick_button.disable_irq();
    g_buttonA_flag = 0;
}

void settings(float &brightness, float &contrast) {
    bool change_brightness = true;
    while (!g_pause_flag) {
        // Take user input and update the arrow
        lcd.clear();
        lcd.printString("Set Bright", 12, 1);
        lcd.printString("Set Contrast", 12, 3);
        Vector2D coord = joystick.get_mapped_coord();
        if (abs(coord.y) > 0.2) {
            change_brightness = !change_brightness;
        }
        lcd.drawSprite(0, (change_brightness) ? 8 : 23, 9, 5, (int *) extra_large_enemy_plane_right);
        if (g_buttonA_flag) {
            joystick_button.disable_irq();
            g_buttonA_flag = 0;
            (change_brightness) ? set_brightness(brightness) : set_contrast(contrast);
            ThisThread::sleep_for(200ms);
            joystick_button.enable_irq();
        }
        lcd.refresh();
        ThisThread::sleep_for(200ms);
    }
    g_pause_flag = 0;
}

bool pause(float &brightness, float &contrast) {
    // To avoid cheating pause all the timers.
    // fire_rate, fast_rate_booster, speed_booster, special_ability;
    auto remaining_fire_rate = fire_rate.remaining_time();
    auto remaining_power_up = power_up.remaining_time();
    auto remaining_special_ability = special_ability.remaining_time();
    auto remaining_survival = survival.remaining_time();
    bool quit = false;
    fire_rate.detach();
    power_up.detach();
    special_ability.detach();
    pause_button.disable_irq();
    g_pause_flag = 0;
    ThisThread::sleep_for(300ms);
    pause_button.enable_irq();
    while (g_pause_flag == 0) {
        Vector2D coord = joystick.get_mapped_coord();
        lcd.clear();
        lcd.printString("Paused", 25, 0);
        lcd.printString("Settings", 25, 3);
        lcd.printString("QUIT", 25, 4);
        if (abs(coord.y) > 0.4) {
            quit = !quit;
        }
        if (quit) {
            lcd.drawSprite(18, 30, 9, 5, (int *) extra_large_enemy_plane_right);
        } else {
            lcd.drawSprite(18, 23, 9, 5, (int *) extra_large_enemy_plane_right);
        }
        if (g_buttonA_flag == 1) {
            if (quit) {
                return true;
            }
            joystick_button.disable_irq();
            ThisThread::sleep_for(300ms);
            g_buttonA_flag = 0;
            joystick_button.enable_irq();
            settings(brightness, contrast);
        }
        lcd.refresh();
        ThisThread::sleep_for(150ms);
    }
    // Reset the timers that were stopped with their remaining times.
    if (remaining_fire_rate > 0ms) {
        fire_rate.attach(&fire_rate_isr, remaining_fire_rate);
    }
    if (remaining_power_up > 0ms) {
        power_up.attach(&power_up_isr, remaining_fire_rate);
    }
    if (remaining_special_ability > 0ms) {
        special_ability.attach(&special_ability_isr, remaining_fire_rate);
    }
    if (remaining_survival > 0ms) {
        survival.attach(&survival_isr, remaining_survival);
    }
    pause_button.disable_irq();
    g_pause_flag = 0;
    return false;
}

void move_supporting_planes(FighterPlane &support_1, FighterPlane &support_2, UserPlane* user) {
    if (support_1.get_active()) {
        support_1.set_position(x_pos - 5, y_pos);
    }
    if (support_2.get_active()) {
        support_2.set_position(x_pos + user->get_width(), y_pos);
    }
}

void play_start_theme(short &count) {
    if (count < 5) {
        buzzer.period_us(1000000.0 / NOTE_E4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 10) {
        buzzer.period_us(1000000.0 / NOTE_G4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 20) {
        buzzer.period_us(1000000.0 / NOTE_A4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 25) {
        buzzer.write(0);
    } else if (count < 30) {
        buzzer.period_us(1000000.0 / NOTE_E4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 35) {
        buzzer.period_us(1000000.0 / NOTE_G4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 45) {
        buzzer.period_us(1000000.0 / NOTE_B4);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 50) {
        buzzer.write(0);
    } else if (count < 55) {
        buzzer.period_us(1000000.0 / NOTE_C5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 60) {
        buzzer.period_us(1000000.0 / NOTE_D5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 70) {
        buzzer.period_us(1000000.0 / NOTE_E5);
        buzzer.pulsewidth_us(buzzer.read_period_us() / 24);
    } else if (count < 80) {
        buzzer.write(0);
    }
    count = (count == 79) ? 0 : count + 1;
}

void start_screen() {
    set<EnemyPlane> planes = {};
    set<EnemyPlane> temp_planes = {};
    set<EnemyBullet> unused = {}; // We do not need bullets but it has to be passed into the function
    short count = 0;
    while (!g_buttonA_flag) {
        play_start_theme(count);
        lcd.clear();
        // Generate planes
        if (rand() % 30 == 0) {
            planes.insert(EnemyPlane(rand() % 75, -5, 'c', 0));
        }
        // Move the planes
        for (EnemyPlane plane : planes) {
            plane.move_plane(0, 1, unused, -100, -100);
            if (get<1>(plane.get_position()) < 49) {
                temp_planes.insert(plane);
                plane.draw_plane();
            }
        }
        unused.clear();
        planes = temp_planes;
        temp_planes.clear();
        lcd.printString("Flying Aces", 12, 2);
        lcd.refresh();
        ThisThread::sleep_for(50ms);
    }
    joystick_button.disable_irq();
    g_buttonA_flag = 0;
    buzzer.write(0);
}

void the_beginning() {
    lcd.clear();
    lcd.printString("The 2 buttons", 0, 0);
    lcd.printString("are for the", 0, 1);
    lcd.printString("pause and ", 0, 2);
    lcd.printString("special", 0, 3);
    lcd.printString("ability", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);
    
    lcd.clear();
    lcd.printString("The LEDs will", 0, 0);
    lcd.printString("tell if the ", 0, 1);
    lcd.printString("ability is", 0, 2);
    lcd.printString("ready or power", 0, 3);
    lcd.printString("up has run out", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);

    lcd.clear();
    lcd.printString("Select your", 0, 0);
    lcd.printString("plane", 0, 1);
    lcd.refresh();
    ThisThread::sleep_for(2s);

    lcd.clear();
    lcd.printString("Bomber has", 0, 0);
    lcd.printString("slow fire", 0, 1);
    lcd.printString("slow move", 0, 2);
    lcd.printString("more health", 0, 3);
    lcd.printString("more bullets", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(3s);

    lcd.clear();
    lcd.printString("Fighter has", 0, 0);
    lcd.printString("fast fire", 0, 1);
    lcd.printString("fast move", 0, 2);
    lcd.printString("less health", 0, 3);
    lcd.printString("less bullets", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(3s);

    lcd.clear();
    lcd.printString("Attacking", 0, 0);
    lcd.printString("lies in", 0, 1);
    lcd.printString("between", 0, 2);
    lcd.refresh();
    ThisThread::sleep_for(2s);
}

void prologue() {
    lcd.clear();
    lcd.printString("Attention, ", 0, 0);
    lcd.printString("pilot. This", 0, 1);
    lcd.printString("is Operation ", 0, 2);
    lcd.printString("Skystorm.", 0, 3);
    lcd.refresh();
    ThisThread::sleep_for(2500ms);
    
    lcd.clear();
    lcd.printString("You will face", 0, 0);
    lcd.printString("enemy", 0, 1);
    lcd.printString("squadron,", 0, 2);
    lcd.printString("changing ", 0, 3);
    lcd.printString("weather", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(2500ms);
    
    lcd.clear();
    lcd.printString("and dynamic", 0, 0);
    lcd.printString("strike zones.", 0, 1);
    lcd.refresh();
    ThisThread::sleep_for(2500ms);
    
    lcd.clear();
    lcd.printString("Stay sharp,", 0, 0);
    lcd.printString("adapt fast,", 0, 1);
    lcd.printString("and complete", 0, 2);
    lcd.printString("your objective", 0, 3);
    lcd.refresh();
    ThisThread::sleep_for(2500ms);
    
    lcd.clear();
    lcd.printString("Dismissed!", 2, 2);
    lcd.refresh();
    ThisThread::sleep_for(2500ms);
    
}

void chpt_1_prologue() {
    lcd.clear();
    lcd.printString("The engines", 0, 0);
    lcd.printString("roar. The sky ", 0, 1);
    lcd.printString("is still. Your", 0, 2);
    lcd.printString("journey begins", 0, 3);
    lcd.printString("here.", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

void chpt_2_prologue() {
    lcd.clear();
    lcd.printString("The wind is", 0, 0);
    lcd.printString("behind you now", 0, 1);
    lcd.printString("do not let it", 0, 2);
    lcd.printString("push you into", 0, 3);
    lcd.printString("trouble.", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

void chpt_3_prologue() {
    lcd.clear();
    lcd.printString("The clouds", 0, 0);
    lcd.printString("grow thick,", 0, 1);
    lcd.printString("hiding friend", 0, 2);
    lcd.printString("and foe.", 0, 3);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

void chpt_4_prologue() {
    lcd.clear();
    lcd.printString("They have", 0, 0);
    lcd.printString("fortified the", 0, 1);
    lcd.printString("skies. Fly ", 0, 2);
    lcd.printString("straight. Dont", 0, 3);
    lcd.printString("look back", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

void chpt_5_prologue() {
    lcd.clear();
    lcd.printString("This place", 0, 0);
    lcd.printString("belongs to no", 0, 1);
    lcd.printString("one. Survive,", 0, 2);
    lcd.printString("or vanish in", 0, 3);
    lcd.printString("the storm.", 0, 4);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

bool final_stage() {
    joystick_button.enable_irq();
    bool endless = false;
    // Let the user choose the final stage
    while (!g_buttonA_flag) {
        lcd.clear();
        lcd.printString("Endless", 12, 1);
        lcd.printString("Last Flight", 12, 3);
        lcd.drawSprite(0, (endless) ? 8 : 23, 9, 5, (int *) extra_large_enemy_plane_right);
        Vector2D coord = joystick.get_mapped_coord();
        if (abs(coord.y) > 0.2) {
            endless = !endless;
        }
        lcd.refresh();
        ThisThread::sleep_for(100ms);
    }
    g_buttonA_flag = 0;
    return endless;
}

void chpt_6_prologue() {
    lcd.clear();
    lcd.printString("With the ", 0, 0);
    lcd.printString("mission ", 0, 1);
    lcd.printString("complete, you", 0, 2);
    lcd.printString("set your", 0, 3);
    lcd.printString("course for", 0, 4);
    lcd.printString("HOME!!!", 0, 5);
    lcd.refresh();
    ThisThread::sleep_for(4s);
}

void ending(bool endless) {
    lcd.clear();
    if (endless) {
        lcd.printString("He never made ", 0, 0);
        lcd.printString("it back.", 0, 1);
        lcd.refresh();
        ThisThread::sleep_for(2s);

        lcd.clear();
        lcd.printString("He never made ", 0, 0);
        lcd.printString("it back. But", 0, 1);
        lcd.printString("somewhere in ", 0, 2);
        lcd.printString("clouds, the ", 0, 3);
        lcd.printString("wings still", 0, 4);
        lcd.printString("fly.", 0, 5);
        lcd.refresh();
        ThisThread::sleep_for(3s);
    } else {
        lcd.clear();
        lcd.printString("You did well", 0, 0);
        lcd.printString("out there", 0, 1);
        lcd.printString("pilot.", 0, 2);
        lcd.refresh();
        ThisThread::sleep_for(3s);
    }
}

void play_buzzer(short type, short count, short counter) {
    int fire_notes[] = {NOTE_C6, NOTE_C5};
    int fire_duration[] = {3, 2};
    int powerup_notes[] = {NOTE_G4, NOTE_C5, NOTE_E5};
    int powerup_duration[] = {3, 3, 3};
    int ability_notes[] = {NOTE_C3, NOTE_C4, NOTE_C5};
    int ability_duration[] = {10, 10, 10};
    int warning_notes[] = {NOTE_A5, REST};
    int warning_duration[] = {5, 10};

    switch (type) {
        case 1:
            if (ability_duration[count] == counter) {
                counter = 0;
                count = (count == 2) ? 0 : count + 1;
                break;
            } else {
                counter++;
            }
            buzzer.period_us(1000000.0 / ability_notes[count]);
            break;
        case 2:
            if (powerup_duration[count] == counter) {
                counter = 0;
                count = (count == 2) ? 0 : count + 1;
                break;
            } else {
                counter++;
            }
            buzzer.period_us(1000000.0 / powerup_notes[count]);
            break;
        case 3:
            if (fire_duration[count] == counter) {
                counter = 0;
                count = (count == 1) ? 0 : count + 1;
                break;
            } else {
                counter++;
            }
            buzzer.period_us(1000000.0 / fire_notes[count]);
            break;
        case 4:
            if (warning_duration[count] == counter) {
                counter = 0;
                count = (count == 1) ? 0 : count + 1;
                break;
            } else {
                counter++;
            }
            buzzer.period_us(1000000.0 / warning_notes[count]);
            break;
    }
    buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
}

void play_notes(int* notes, chrono::milliseconds* durations, short len) {
    for (int i = 0; i < len; i++) {
        if (notes[i] != REST) {
            buzzer.period_us(1000000.0 / notes[i]);
            buzzer.pulsewidth_us(buzzer.read_period_us() / 4);
        } else {
            buzzer.write(0);
        }
        ThisThread::sleep_for(durations[i]);
    }
}

void play_end(bool success) {
    if (success) {
        int notes[7] = {NOTE_C5, NOTE_G4, NOTE_E4, NOTE_C4, REST, NOTE_G3, NOTE_C4};
        chrono::milliseconds durations[7] = {200ms, 200ms, 250ms, 400ms, 100ms, 300ms, 500ms};
        play_notes(notes, durations, 7);
    } else {
        int notes[5] = {NOTE_E4, NOTE_D4, NOTE_C4, REST, NOTE_G3};
        chrono::milliseconds durations[5] = {250ms, 250ms, 400ms, 100ms, 400ms};
        play_notes(notes, durations, 5);
    }
    
}