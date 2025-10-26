# Joystick_Coordinate_Test

The Joystick and N5110 libraries are not written by me. Link for the libraries are provided below

Required Libraries:     Joystick : https://github.com/ELECXJEL2645/Joystick  
                        N5110    : https://github.com/ELECXJEL2645/N5110

MBED Studio Version:    1.4.1  
MBED OS Version:        6.12.0  
Board:	                NUCLEO L476RG

## Introduction

This projects re-creates Capcom's 1942 arcade game on a PCD8544 LCD display with a joystick and buttons to play the game. The circuit board is shown below.

<img width="3952" height="2029" alt="Annotated Hardware Setup" src="https://github.com/user-attachments/assets/101833f9-0f8b-4fe4-a1cb-885c9d0388e0" />

## Game Explained

The user will play 5 different chapters that increase in difficulty with a boss at the end of each. At the end, the user will have the choice of two endings: continue in endless mode or a time limited mode. The user is presented with win or loss screen at the end of the game.
The user can choose from 3 different planes:
- A Bomber has the most health and strongest firing capability but has the slowest speed and rate of fire.
- Fighter planes have the fastest rate of fire and speed but has the least health and weakest firing capability.
- Attacking planes lie in between both.

Each plane has a unique special ability that can be used and regenerates every 20 seconds. The Bomber drops a bomb that clears the entire screen of enemies. The Attacking plane gets 10 piercing rounds. The Fighter plane calls for a large number of backup planes that move from the bottom of the screen to the top while firing.

The game has clouds which can conceal enemies and wind which can affect user speed, bullet directions and speed. The score, health bar and lives are shown on the right side of the screen. The most notable features of the game are:
- Smart targeting, which starts in chapter 3, allowing the enemies to predict the user's movement and fire according to where the user is moving.
- Heatmaps recording where the user is active the most thereby spawning enemies accordingly.
- Enemy formations which spawns the enemy in 16 different formations with each formation having a formation leader that the other planes follow. Some formations move along a fixed path while others move towards the user.
- Reformation of enemy plane when one of the planes in the formation is destroyed.
- Merging of separate formations into one which causes the planes of the two formations to move into a formation of one.
- A buzzer playing noises when the user is destroyed, fires their weapon, collects a booster or is low on health.

The game has 8 booster shown below. The sprites are coloured here; however, the screen only displays black and white.

<img width="3317" height="2475" alt="Annotated Boosters" src="https://github.com/user-attachments/assets/f7a6214b-4f66-450b-8b27-38fe8c38c6ac" />
