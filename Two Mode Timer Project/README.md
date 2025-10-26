# Two Mode Timer
## Introduction
This is a project to make a timer using Intel Quartus and verifying the modules using ModelSim Altera. The goal of the project is to build a timer that can count both upwards (stopwatch) up to 99.99 seconds and downwards (timer) from any minute between 8.00 and 1.00. Another objective was to make the project using only structural logic (apart from D Flip Flops). The challenge was to make the timer strictly using the structure below.

<img width="4431" height="2335" alt="High Level Structure" src="https://github.com/user-attachments/assets/4744a651-f279-4bb1-a1c7-f784b8f8c571" />

## Features
- A switch allows the user to change between Mode A (Stopwatch) and Mode B (Timer).
- A reset button allows the user to reset the timer and a start button lets the user start and stop the timer.
- In Mode A, the timer counts from 00.00 to 99.99 and stops when it reaches 99.99.
- In Mode B, the timer counts down till 0.00 from X.00 where X is any minute that the user can set from 8 to 1 using 3 switches. The timer automatically stops at 0.00.
- In Mode B, switching the start time (e.g., from 1.00 to 3.00) will automatically reset the timer.
- The LED flashes during the last 10 seconds of both Mode A and Mode B and remains ON when the timer finishes in Mode B or when the stopwatch reaches 99.99 in Mode A.
