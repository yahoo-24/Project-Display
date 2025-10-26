/*        Flying aces
*        ========================
*
*        Function:               A game based on Capcom's 194X series
*        Required Libraries:     Joystick : https://github.com/ELECXJEL2645/Joystick
*                                N5110    : https://github.com/ELECXJEL2645/N5110
*
*        Authored by:            Dr Yahia
*        Date:                   02/2025
*        Collaberators:          
*        Version:                1.0
*        Revision Date:          05/2025 
*        MBED Studio Version:    1.4.1
*        MBED OS Version:        6.12.0
*        Board:	                 NUCLEO L476RG  */

#include "Joystick.h" 
#include <cmath>
#include <cstdio>
#include <iterator>
#include <set>
#include <tuple>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <algorithm>
#include "Sprites.h"
#include "N5110.h"
#include "Globals.h"
#include "UserPlane.h"
#include "Enemies.h"
#include "Booster.h"
#include "BossPlane.h"
#include "Funcs.h"

int main(){
    srand(time(0)); // Initialise the random number generator

    joystick_button.fall(&joystick_button_isr);
    joystick_button.mode(PullUp);
    pause_button.mode(PullUp);
    special_ability_button.fall(&special_button_isr);
    fire_rate.attach(&fire_rate_isr, 500ms);

    joystick.init();

    float contrast = 0.55;
    float brightness = 0.15;

    lcd.init(LPH7366_6);        //initialise for LPH7366-1 LCD (Options are LPH7366_1 and LPH7366_6)
    lcd.setContrast(contrast);      //set contrast to 55%
    lcd.setBrightness(brightness);     //set brightness to 50% (utilises the PWM)

    map<char, set<tuple<float, float>>> bullet_set_float = {
        {'L', {}},
        {'S', {}},
        {'R', {}}
    };

    bool user_hit = false;

    int size_x, size_y;

    set<EnemyBullet> enemy_fire = {};

    start_screen();
    the_beginning();

    // The button was disabled so you do not accidentally choose a plane before the menu
    // It is also there incase the button detects 2 clicks instead of 1.
    joystick_button.enable_irq();
    UserPlane* user_plane = get_choice(plane_menu());
    set<tuple<int, int>> plane_pixels = update_plane_pixels(user_plane);

    int enemy_gen = 0;

    PowerUps power_up_arr[8] = {
        {false, Booster("Health"), 250},
        {false, Booster("Fast Rate"), 300},
        {false, Booster("Speed"), 300},
        {false, Booster("Deflect"), 350},
        {false, Booster("Invincible"), 500},
        {false, Booster("Scatter"), 400},
        {false, Booster("Life"), 1000},
        {false, Booster("Support"), 400}
    };

    char special_ability_type = 'x'; // The user plane method will set it to the value based on the plane type.

    float heatmap[4] = {1, 1, 1, 1};
    UserTracker user_tracker;
    float average_displacement_x = 0;
    float average_displacement_y = 0;

    g_special_ability_timer = 1; // Duration before the abilities reset
    float special_ability_counter = 0; // Used in fighter plane and attacking plane

    set<Formation> formations = {};
    set<Formation> formations_temp = {};
    map<short, set<Formation>> formation_classifier = {
        {1, {}},
        {2, {}},
        {3, {}}
    };

    int min_points[17] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0};

    set<Cloud> clouds = {};

    pause_button.fall(&pause_button_isr);

    FighterPlane supporting_plane_1 = FighterPlane();
    FighterPlane supporting_plane_2 = FighterPlane();

    struct State {
        char* stage_name;
        int stage_num;
        StageParameters stage_parameters;
        int next_stage;
    };
    State stages_fsm[6] = {
        {(char*)"Calm Before the Storm", 1, {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, 0, {0, 100, 100, 100, 100}, 70, 10000, 50, false}, 1},
        {(char*)"Tailwind Turbulence", 2, {{0, 0, 0, 0, 0}, {0, -0.2, -0.3, 0, 0}, 0, {0, 15, 30, 100, 100}, 65, 10000, 75, false}, 2},
        {(char*)"Fog of War", 3, {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, 0, {0, 180, 100, 100, 100}, 55, 90, 100, false}, 3},
        {(char*)"Storm of Iron", 4, {{-0.1, -0.2, 0, 0, 0}, {-0.1, -0.2, 0, 0, 0}, 0, {0, 25, 100, 180, 100}, 50, 100, 125, true}, 4},
        {(char*)"No Mans Air", 5, {{-0.3, 0.1, 0.4, -0.3, 0.1}, {0.3, -0.3, 0.4, 0.3, -0.1}, 0, {0, 25, 50, 60, 160}, 40, 45, 175, true}, 5},
        {(char*)"The Long Flight Home", 6, {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, 0, {0, 1000000, 50, 60, 160}, 60, 50, 10000, true}, 6}
    };
    // Stage parameters for the 6 stages are shown above
    
    bool success = false;

    int state_index = 0; // Index for FSM
    int total_score = 0; // Score stroed by the plane resets so this variable holds them before reseting
    bool quit = false; // Checks if the user clicks Quit in the pause menu
    bool endless = false; // Endless mode or Time mode
    prologue();

    while (!user_plane->game_over() && !quit) {
        State state = stages_fsm[state_index];
        StageParameters stage_param = state.stage_parameters;
        char* stage_name = state.stage_name;

        // Print the stage name and chapter number
        lcd.clear();
        char buffer[14] = {0};
        sprintf(buffer, "Chapter %d", state.stage_num);
        lcd.printString(buffer, 2, 2);
        lcd.refresh();
        ThisThread::sleep_for(3s);
        lcd.clear();
        if (state.stage_num > 2 && state.stage_num != 6) {
            lcd.printString(stage_name, 2, 2);
        } else {
            if (state.stage_num == 1) {
                lcd.printString("Calm Before", 2, 2);
                lcd.printString("the Storm", 2, 3);
            } else if (state.stage_num == 2) {
                lcd.printString("Tailwind", 2, 2);
                lcd.printString("Turbulence", 2, 3);
            } else {
                lcd.printString("The Long ", 2, 2);
                lcd.printString("Flight Home", 2, 3);
            }
        }
        lcd.refresh();
        ThisThread::sleep_for(3s);

        // Print the stage intro
        switch(state.stage_num) {
            case 1:
                chpt_1_prologue();
                break;
            case 2:
                chpt_2_prologue();
                break;
            case 3:
                chpt_3_prologue();
                break;
            case 4:
                chpt_4_prologue();
                break;
            case 5:
                chpt_5_prologue();
                break;
            case 6:
                chpt_6_prologue();
                if (!endless) {
                    survival.attach(&survival_isr, 60s);
                }
                break;
        }

        // Set the wind and move the index to the next element
        wind_x = stage_param.wind_x[0];
        wind_y = stage_param.wind_y[0];
        stage_param.wind_index++;
        BossPlane boss = BossPlane(1);
        bool boss_battle = false; // If false then it initialises the boss for the battle when min points reached
        bool stage_complete = false; // Used to prevent enemy spawn before round ends
        g_round_end = 0; // Causes a 5 second delay before the round ends

        // Counters for the notes and durations for the buzzer
        // The bool values is needed to check if it is necessary to play a sound
        bool weapon_fired = false;
        short fire_note_counter = 0;
        short fire_duration_counter = 0;
        short warning_note_counter = 0; // Warning does not need a bool value as it uses the user's hp
        short warning_duration_counter = 0;
        bool powerup_collected = false;
        short powerup_note_counter = 0;
        short powerup_duration_counter = 0;
        bool ability_used = false;
        short ability_note_counter = 0;
        short ability_duration_counter = 0;

        while (!user_plane->game_over() && !g_round_end && !quit) {
            // read the joystick to get the x- and y- values
            Vector2D coord = joystick.get_mapped_coord();     
            lcd.clear();  // clear buffer at the start of the loop

            weapon_fired = check_if_user_fired_weapon(user_plane, special_ability_type, special_ability_counter, bullet_set_float, supporting_plane_1, supporting_plane_2);

            // If the user takes the support plane booster then reset the planes to their default
            if (user_plane->get_support()) {
                supporting_plane_1.reset_plane();
                supporting_plane_2.reset_plane();
                user_plane->set_support(false);
            }

            // Make sure we are not out of the range of the wind array
            if (stage_param.wind_index != 5) {
                // If we have exceeded the minimum point threshold then move to the next wind in the array
                if (user_plane->get_points() >= stage_param.wind_point_threshold[stage_param.wind_index]) {
                    wind_x = stage_param.wind_x[stage_param.wind_index];
                    wind_y = stage_param.wind_y[stage_param.wind_index];
                    stage_param.wind_index++;
                }
            }

            // Set the special ability type.
            // Fighter plane gets friendly plane support
            // Attacking plane gets piercing bullets
            // Bomber get an atomic bomb
            // Check if the button for the special ability is clicked.
            ability_used = false;
            if (g_special_ability_timer && g_special_flag) {
                g_special_ability_timer = 0;
                ability_led = 0;
                special_ability.attach(&special_ability_isr, 20s);
                user_plane->special_ability(special_ability_type);
                ability_used = true;
            } else if (g_special_ability_timer) {
                ability_led = 1; // Ability ready but not used
            }
            g_special_flag = 0;

            // Check what type of special ability and implement it. 'x' is none.
            if (special_ability_type == 'B') {
                user_plane->score_points(atomic_bomb(formations, enemy_fire));
                special_ability_type = 'x';
            } else if (special_ability_type == 'F') {
                fighter_plane_support(special_ability_counter, special_ability_type, bullet_set_float);
            }

            if (!stage_complete) {
                generate_enemy_planes(user_plane->get_points(), formations, min_points, stage_param.spawn_intensity, heatmap);
            }

            formations_temp.clear();
            for (Formation formation : formations) {
                if(!formation.move_planes(user_hit, user_plane, plane_pixels, bullet_set_float, special_ability_type, enemy_fire, (stage_param.predictive_targeting) ? average_displacement_x : -100, average_displacement_y, supporting_plane_1, supporting_plane_2)) {
                    classify_formation(formation_classifier, formation);
                    formations_temp.insert(formation);
                }
            }
            formations = formations_temp;
            formation_merger(formation_classifier[0], formations, heatmap);
            formation_merger(formation_classifier[1], formations, heatmap);
            formation_merger(formation_classifier[2], formations, heatmap);
            formation_classifier[0].clear();
            formation_classifier[1].clear();
            formation_classifier[2].clear();

            powerup_collected = update_boosters(power_up_arr, plane_pixels, user_plane);

            if (reset_plane_to_default == 1) {
                // The booster has run out
                reset_plane_to_default = 0;
                power_up_led = 0;
                user_plane->reset_parameters();
            }

            if (!boss_battle && user_plane->get_points() >= stage_param.point_threshold) {
                boss = BossPlane(state.stage_num);
                boss_battle = true;
            } else if (boss_battle && !boss.get_defeated()) {
                // Move the boss and see if the user crashed into the boss
                user_hit = boss.move(plane_pixels, user_plane);
                // Did the user destroy the boss
                if (boss.destroyed(bullet_set_float)) {
                    stage_complete = true;
                    round_end.attach(&round_end_isr, 5s);
                }
                if (!user_hit) {
                    // Fire rounds and check if it hits the user
                    boss.fire_rounds(average_displacement_x, average_displacement_y);
                    user_hit = boss.update_fire(plane_pixels, user_plane);
                }
            }

            update_user_bullets(bullet_set_float);

            lcd.drawSprite(x_pos, y_pos, user_plane->get_height(), user_plane->get_width(), user_plane->sprite);
            check_movement(coord, user_plane->get_speed_factor(), user_tracker);
            move_supporting_planes(supporting_plane_1, supporting_plane_2, user_plane);
            plane_pixels = update_plane_pixels(user_plane);
            adjust_heatmap(heatmap);
            average_displacement_x = user_tracker.get_average_x();
            average_displacement_y = user_tracker.get_average_y();

            update_enemy_fire(enemy_fire, user_hit, user_plane, plane_pixels, bullet_set_float, supporting_plane_1, supporting_plane_2);

            // Draw the UI on the right of the screen.
            UI::draw_score(user_plane->get_points());
            UI::draw_hitpoints(user_plane->get_hitpoints(), user_plane->get_max_hitpoints());
            UI::draw_lives(user_plane->get_lives());
            generate_cloud(clouds, stage_param.cloud_intensity);
            move_clouds(clouds);

            if (g_pause_flag == 1) {
                quit = pause(brightness, contrast);
                ThisThread::sleep_for(400ms);
                pause_button.enable_irq();
            }

            check_user_hit(user_hit, formations, bullet_set_float, enemy_fire);

            if (state.stage_num == 6 && !endless && g_survival) {
                // Did the user surviv the allocated time in the time-limited mode
                g_survival = 0;
                stage_complete = true;
                round_end.attach(&round_end_isr, 5s);
            }

            // Print the buzzer sounds. Highest priority goes to the ability then powerup, fire, low Hp
            if (ability_used || ability_note_counter || ability_duration_counter) {
                powerup_note_counter = 0;
                powerup_duration_counter = 0;
                fire_note_counter = 0;
                fire_duration_counter = 0;
                warning_note_counter = 0;
                warning_duration_counter = 0;
                play_buzzer(1, ability_note_counter, ability_duration_counter);

            } else if (powerup_collected || powerup_note_counter || powerup_duration_counter) {
                fire_note_counter = 0;
                fire_duration_counter = 0;
                warning_note_counter = 0;
                warning_duration_counter = 0;
                play_buzzer(2, powerup_note_counter, powerup_duration_counter);

            } else if (weapon_fired || fire_note_counter || fire_duration_counter) {
                warning_note_counter = 0;
                warning_duration_counter = 0;
                play_buzzer(3, fire_note_counter, fire_duration_counter);

            } else if (user_plane->get_hitpoints() <= 20 || warning_note_counter || warning_duration_counter) {
                play_buzzer(4, warning_note_counter, warning_duration_counter);

            } else {
                buzzer.write(0); // No sound to be played
            }
            
            ThisThread::sleep_for(30ms);
        }
        buzzer.write(0);
        if (state.stage_num == 6) {
            // If the bonus stage is done then play the ending
            success = true;
            ending(endless | user_plane->game_over());
            break;
        }
        // Move to the next stage if the stage is complete except for stage 5 where you also have to present the option of choosing the ending
        if (stage_complete) {
            if (state.stage_num == 5) {
                joystick_button.disable_irq();
                ThisThread::sleep_for(100ms);
                endless = final_stage();
            }
            state_index = state.next_stage;
            total_score += user_plane->get_points();
            user_plane->reset_points();
            decay_heatmap(heatmap);
        }
    }
    game_over_screen(user_plane, total_score, success);
    delete user_plane;
    g_round_end = 0;
    round_end.attach(&round_end_isr, 10s);
    while (!g_round_end) { play_end(success); }
    lcd.turnOff();
    ability_led.write(0);
    power_up_led.write(0);
    buzzer.suspend();
}
