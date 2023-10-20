**Bin detection and picking project** 🤖🦾🚮🗑️
======
This repo is the final project for the Fall 2019 Robotics course at NTU (National Taiwan University).
It aimed to teach a Dobot Magician robot arm to play dominoes with a human.
- A camera is used to take pictures of the board and to get the plays made by the human opponent. 
- The game is modeled by a finite state machine (FSM) representing the state at every step.
An approach with a custom ***decision tree*** was used to select the best strategy at every step. 

![animation](./images/animation.gif)

## I) Key Modules used

|                                    *Computer vision*                                    |                                   *Artificial Intelligence*                                    |                                                      *Robot Control*                                                      |
|:---------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------:|
| <img src="https://opencv.org/wp-content/uploads/2019/02/opencv-logo-1.png" width="100"> | <img src="https://image.freepik.com/free-vector/brain-logo-template_15146-28.jpg" width="150"> | <img src="https://yt3.ggpht.com/a-/AN66SAxJ4HOFNSU5S1MwTgzhQCdwPNGEixYalis6ZA=s900-mo-c-c0xffffffff-rj-k-no" width="150"> |
|                                     OpenCV package                                      |                                  **Rule bases Decision Tree**                                  |                                                      PyDobot package                                                      | 
|                       Detect the dominoes and read the game state                       |                         Analyse the game state and find the best move                          |                                          Pick up, move and release the dominoes                                           |

## II) The result in video