#ifndef FUNCS_H
#define FUNCS_H

#include "UserPlane.h"
#include "Enemies.h"
#include "Booster.h"
#include "Notes.h"

UserPlane* get_choice(int choice);

class UserTracker {
    private:
        float _displacements_x[30] = {0};
        float _displacements_y[30] = {0};
        int _index_x = 0;
        int _index_y = 0;
        float _sum_x = 0;
        float _sum_y = 0;

        void update_index_x();

        void update_index_y();

    public:
        UserTracker();

        void update_x(float val_x);

        void update_y(float val_y);

        float get_average_x();

        float get_average_y();
};

void check_movement(Vector2D coord, float sf, UserTracker &tracker);

set<tuple<int, int>> update_plane_pixels(UserPlane* plane);

void joystick_button_isr();

void special_button_isr();

void fire_rate_isr();

void special_ability_isr();

void pause_button_isr();

void round_end_isr();

void survival_isr();

bool check_if_user_fired_weapon(UserPlane *user_plane, char &special, float &counter, map<char, set<tuple<float, float>>> &bullet_set_float, FighterPlane &support_1, FighterPlane &support_2);

void generate_enemy_planes(int points, set<Formation> &formations, int* min_points, int factor, float* heatmap);

struct PowerUps{
    bool active;
    Booster booster;
    int probability;
};

bool generate_booster(int probability);

bool update_boosters(PowerUps* booster_arr, set<tuple<int, int>> pixels, UserPlane *user);

void update_user_bullets_helper(map<char, set<tuple<float, float>>> &bullet_set_float, char type);

void update_user_bullets(map<char, set<tuple<float, float>>> &bullet_set_float);

void update_enemy_fire(set<EnemyBullet> &enemy_fire, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, FighterPlane &support_1, FighterPlane &support_2);

void check_user_hit(bool &user_hit, set<Formation> &formations, map<char, set<tuple<float, float>>> &bullet_set, set<EnemyBullet> &enemy_fire);

void game_over_screen(UserPlane *user_plane, int score, bool success);

int atomic_bomb_helper_function(Formation &formation);

int atomic_bomb(set<Formation> &formations, set<EnemyBullet> &enemy_fire);

void fighter_plane_support(float &counter, char &type, map<char, set<tuple<float, float>>> &bullet_set);

void adjust_heatmap(float *heatmap);

int plane_menu();

void decay_heatmap(float* heatmap);

class Cloud {
    private:
        float _x;
        float _y;
    public:
        Cloud();

        void draw_cloud();

        bool move_cloud();

        bool operator<(const Cloud& other) const;
};

void generate_cloud(set<Cloud> &clouds, int probability_factor);

void move_clouds(set<Cloud> &clouds);

void set_brightness(float &brightness);

void set_contrast(float &contrast);

void settings(float &brightness, float &contrast);

bool pause(float &brightness, float &contrast);

void move_supporting_planes(FighterPlane &support_1, FighterPlane &support_2, UserPlane* user);

struct StageParameters {
    float wind_x[5];
    float wind_y[5];
    int wind_index;
    int wind_point_threshold[5];
    int spawn_intensity;
    int cloud_intensity;
    int point_threshold;
    bool predictive_targeting;
};

void play_start_theme(short &count);

void start_screen();

void the_beginning();

void prologue();

void chpt_1_prologue();

void chpt_2_prologue();

void chpt_3_prologue();

void chpt_4_prologue();

void chpt_5_prologue();

bool final_stage();

void chpt_6_prologue();

void ending(bool endless);

void play_buzzer(short type, short count, short counter);

void play_notes(int* notes, chrono::milliseconds* durations, short len);

void play_end(bool success);

#endif