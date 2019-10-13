
Smart Shoes: An innovative way to fix improper gait

  My project goal is to measure the movement of the user’s feet, to determine if their gait was proper, display the collected data or allow for real time correction of the feet, and provide a program that professionals could use to analyze their data.
  
  The data will be shown in the form of graphs and an image of shoes that will match your feet. It will be designed in the form of a small module that can be attached to a pair of shoe to measure the movement that they are. The purpose was to aid doctors and medical professionals with a program that displays easily understandable data. It was also to help people with gait disorders fix their gait with live angle detection and alerting them via beep. I aimed to help people with feet that point inward or outward with this project. 
  
  I started by designing the entire add-on consisting of various different modules on a breadboard. The design contains an Arduino, gyroscope/accelerometer, and a bluetooth module. I tested and improved the design on the breadboard. When I felt the design was optimized, I soldered the modules onto a PCB board. After testing the design on the smaller board, I drilled a hole into the sole of the shoe and bolted the module onto the sole. 
  
  When I knew the physical product was done, I turned my attention to the competitive part of this project. Due to each program having its advantages, I used both Arduino and a PC. Arduino was used to collect data from the gyroscope/accelerometer. Arduino sent it to to the PC via bluetooth and the PC would collect it so that it could display the data. The Shoe motion analyzer would filter the data and use it in the various displays of data. It displays data by, showing live graphs of the feet in the 3 axes, a pair of shoes that turn to the position your feet are currently in, and a box that maps your footsteps and the angles your feet were at the time. To map footsteps, I created an algorithm that found the peak and base of spike from the acceleration of the Z axis. I subtracted the two values and divided it by the amount of points in-between resulting in the slope between the points. If this number passed an acceptable point it was considered as a step and marked on the graph. 
  
  At the end of the project, I felt that I had satisfied my goals; creating a compact board that could effectively measure the angle of your feet in 3 axis and measure acceleration as well. The shoe suffered no lack of comfort or feel in the addition of the module. 
  
