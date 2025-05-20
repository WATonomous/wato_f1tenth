### Welcome to Micro Autonomy!

We're watonomous F1Tenth team (1/10th scale autonomous racing). We compete in F1Tenth racing competitions held at research conferences globally throughout the year.

# Onboarding Steps
Apart from the general WATOnomous onboarding, there are some Micro-specific things you'll have to read & learn before contributing. Don't worry, it's a lot lighter than the onboarding assignnmet :)

### Background Theory
1. Course Syllabus:  https://docs.google.com/spreadsheets/d/1PaFYG7XC_XQ3ExdQGb-S8oJzzixoMOVjh4L1RjW0gT0/edit?gid=0#gid=0o

Although not mandatory to study this in great detail, we **highly** reccommend skimming all the slides up to Module D. You can revisit this later when working on a specific component, but getting a general idea of what SLAM / Planning / Controls mean will be beneficial. 

2. Competition Rules: https://iros2024-race.f1tenth.org/rules.html

A lot of our software architecture choices are based on the rules.

### Environment Setup

Our infrastructure consists of 1) AUTODrive physics simulator connected to 2) Docker environment. As such, you'll have to go through the `infra-setup.md` and follow the steps.

## Your First Mini Component

To verify that the environment setup works properly to familiarize yourself with how ROS2 works, please make a PR for a rosnode that:

1. Subscribes to the IMU and prints IMU data onto the console
2. Publish a throttle command to make the car move forward

References: https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-guide/

note: This rosnode skeleton you'll most likely be building ontop of for your actual task, so getting this working well (no build errors, etc.) will save a lot of time in the future.

### Good Luck! 
