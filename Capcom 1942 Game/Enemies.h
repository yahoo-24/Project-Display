#ifndef ENEMIES_H
#define ENEMIES_H

#include <tuple>
#include <set>
#include <map>
#include "Sprites.h"
#include "Globals.h"
#include "UserPlane.h"
#include "mbed.h"

class EnemyBullet {
    private:
        float _x;
        float _y;
        float _x_move;
        float _y_move;
        bool _dynamic;

    public:
        EnemyBullet(int x, int y, float x_move, float y_move, bool dyn=false);

        void draw_bullet();

        bool operator<(const EnemyBullet& other) const;

        void move();

        tuple<int, int> get_position();
};

void heatmap_sum(float *heatmap, int* cum_sum);
int generate_random_position(float *heatmap);
tuple<double, double> predict_location(float pos_x, float pos_y, float e_pos_x, float e_pos_y, float mx, float my);

class EnemyPlane {
    public:
        EnemyPlane(float x, float y, char type, int rounds=rand() % 4);

        EnemyPlane();

        tuple<float, float> get_position();

        bool operator<(const EnemyPlane& other) const;

        char get_type();

        void move_plane(float x, float y, set<EnemyBullet> &enemy_fire, float mx, float my);

        void fire_rounds(set<EnemyBullet> &enemy_fire, float mx, float my);

        void draw_plane();

        int get_rounds();

    private:
        float _position_x;
        float _position_y;
        char _type;
        int _rounds; // Can only fire 3 rounds
};

bool bullet_hit(float x, int y, float xb, int yb, int side);

bool find_bullet_helper(map<char, set<tuple<float, float>>> &bullet_set, int x, int y, char type, char special);

bool find_bullet(map<char, set<tuple<float, float>>> &bullet_set, int x, int y, char special);

bool check_collisions(EnemyPlane &element, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char special, FighterPlane &support_1, FighterPlane &support_2);

class Formation {
    private:
        tuple<int, int> _offset[5];
        EnemyPlane _followers[5];
        EnemyPlane _leader;
        bool _fixed_path;
        float _x_move;
        float _y_move;
        short _num_of_planes;
        bool destroyed[6];
        short _formation_type;
        int _theta[3] = {90, 90, 90};
        static long id_counter;
        int _unique_id;
        short _remaining;
    
    public:
        Formation();

        Formation(short formation_type, float *heatmap, int x=-99);

        static Formation merge(Formation& form1, const Formation& form2, float *heatmap);

        void vee_formation(int x, int y);

        void echelon_right(int x, int y);

        void echelon_left(int x, int y);

        void double_arrow(int x, int y);

        void diamond(int x, int y);

        void vertical_trail(int x, int y);

        void horizontal_trail(int x, int y);

        void heavy_left(int x, int y);

        void heavy_right(int x, int y);

        void staggered_left(int x, int y);

        void staggered_right(int x, int y);

        void solo(int x, int y);

        void zigzag();

        void elliptical();

        void solo_ambush(int x);

        void targeting_solo(int x, int y);

        void wingman(int x, int y);

        void zigzag_move(int x, int y);

        void elliptical_move(float x, float y, short i);

        void ambush_move(int y);

        void targeting_move(int x, int y);

        void generate_enemies(int x, int y);

        bool move_planes(bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char &special, set<EnemyBullet> &enemy_fire, float mx, float my, FighterPlane &support_1, FighterPlane &support_2);

        bool move_followers(tuple<float, float> pos, bool &user_hit, UserPlane *user_plane, set<tuple<int, int>> &plane_pixels, map<char, set<tuple<float, float>>> &bullet_set, char &special, set<EnemyBullet> &enemy_fire, float mx, float my, FighterPlane &support_1, FighterPlane &support_2);

        bool operator<(const Formation& other) const;

        EnemyPlane get_leader();

        EnemyPlane* get_followers();

        short get_num_planes();

        short get_formation_type();

        float get_x_move();

        float get_y_move();

        tuple<float, float> get_position();

        short get_remaining();

        void draw_plane(EnemyPlane &plane);
};

tuple<double, double> predict_location(float pos_x, float pos_y, float e_pos_x, float e_pos_y, float mx, float my);

void classify_formation(map<short, set<Formation>> &formation_merge, Formation &formation);

void formation_merger(set<Formation> &formations_to_merge, set<Formation> &formations, float* heatmap);

#endif