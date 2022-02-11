<div>
<img src=
"/assets/Dronetech.png" 
         alt="Dronetech logo" 
         align="right" 
         width="70">
<img src=       
"/assets/titleLogo.png" 
         alt="technex logo" 
         align="left" 
         width="80">
</div>
<br><br>

<h1 align="center"> DroneTech-2022 </h1>
<br>

<div align = "center">
<img src="/assets/Drone.gif" alt="Drone" align="center">
</div>
         
## Table of contents
<details>
  <ol>
    <li>
       <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#round-1">Round 1 : Quiz Evaluations</a>
    </li>
    <li>
     <a href="#round-2">Round-2 : Automate the Drone</a>
     <ul>
        <li><a href="#problem-statement">Problem Statement</a></li>
        <li><a href="#drone-specifications">Drone Specifications</a></li>
        <li><a href="#points-to-note">Points to Note</a></li></li>
        <li>
         <a href="#installation-procedures">Installation Procedures</a>
          <ul>
           <li><a href="#webots-simulator">WeBots Simulator</a></li>
           <li><a href="#important-note">Important Note</a></li>
         </ul>
        </li>
        <li><a href="#setup-requirements">Setup Requirements</a></li>
        <li><a href="#initialisation">Initialisation</a></li>
    </ul>
   </li>
    <li><a href="#scoring">Scoring</a></li>
    <li><a href="#instructions-regarding-submissions">Instructions Regarding Submissions</a></li>
  </ol>
</details>

# Overview
All over India, students are waiting in a hope to be called back to the campus. Now that their voices went unheard multiple number of times, they have decided to break the doors of the institute. But there are some moulds inside the main door, in which color sensors are placed. In order to unlock it, the moulds need to be filled with shapes of appropriate color. These shapes are hidden at various places in our world and hence, the time has come for our friend Mavic Pro to help find those shapes. But before taking his help, you must know that the only thing our mini friend is afraid of is collisions :(


<p align="center">
 <img src="/assets/image4.png">
 <p align="center">
</p>
<p align="center">
 <img src="/assets/image5.png">
 <p align="center">
 <i>Glimpses of the world</i><br> 
</p>

To solve this major problem of college reopening, we cannot take any risk and hence, we need to test you in two rounds. Round 1 requires submitting a quiz on D2C which will be based on drone dynamics, so that we can ensure that you understand our friend very well, and as for the second round, you need to get your hands dirty with a simulator.

# Round 1

Quiz Based Evaluation

Round 1 will be a quiz on D2C with the following pattern:
- Quiz will be MCQ based with total of 15 questions.
- There will be two sections, A and B:
           <ul>
             <li>Section A : MCQ with single correct answers </li>
             <li>Section B : TEXT-BASED questions with a single-word correct answer</li>
           </ul>
        
- Section A questions will carry 2 marks each and Section B questions will carry 4 marks each.
- No negative marking.
- No partial marking in section B for incomplete text.
- Quiz will be of 40 minutes.

      
# Round 2

Autonomous Drone navigation, colour and shape detection

### **<ins>Problem Statement</ins>**
The simulator we will be using is WeBots simulator. Your drone needs to scan a few QR codes in order to find the shapes. Perform the following tasks with the drone and go enjoy your college life :)
1. Using the camera provided on the drone, scan a QR code placed right in front of it. 
<p align="center">
 <img src="/assets/image2.png">
 <p align="center">
 <i>This is where the drone will spawn</i><br> 
</p>

2. This QR code provides the name of the shape of the next box as well as its RGB values.
3. Locate the next box using the above information and store its world coordinates.
<p align="center">
 <img src="/assets/image6.png">
 <p align="center">
</p>

4. Also scan the QR code placed on this box which again provides the name of the next shape and its RGB values.
5. Locate the next box using the above information and store its world coordinates.
6. Keep moving to the next box until you reach a point where the QR code contains coordinates instead of shape name and RGB values.
<p align="center">
 <img src="/assets/image7.png">
 <p align="center">
 <i>This QR code contains the world coordinates of the banner</i><br> 
</p>

7. Now, reach those coordinates where you will find a banner lying on the floor with various white shapes drawn over it.
<p align="center">
 <img src="/assets/image8.png">
 <p align="center">
</p>

8. If the colored boxes are the vertices of a polygon (in the same order in which you are asked to traverse them), then out of the various shapes drawn on the banner, the resulting polygon (scaled down) is where your drone needs to land.
Landing of the drone on the correct shape determines the successful completion of the problem statement.

### **<ins>Drone Specifications:</ins>**
1. The drone used is Dji Mavic 2 Pro.
2. It has one downward facing camera.
3. It has three distance sensors (left, right and front).
<p align="center">
 <img src="/assets/image9.png">
 <p align="center">
 <i>Dji Mavic 2 Pro</i><br> 
</p>

4. In depth specifications of the drone are [here](https://www.cyberbotics.com/doc/guide/mavic-2-pro?version=develop#mavic2pro-field-summary).
5. To know more about the sensors used in the drone, click [here](https://github.com/cyberbotics/webots/tree/released/docs/reference)

### **<ins>Points to Note</ins>**
1. **You are not allowed to modify the world or the properties of the drone in any manner.**
2. **The arena given in this repo is only for practice. Final arena will have the exact same configuration except that the number and position of the boxes, and the shapes on the final banner will differ. Top surface of the boxes in the final arena will also be either square or circle.**
3. **If the drone directly lands on a shape (luckily the correct shape) on the banner without scanning all the QR codes, it will be considered as an INVALID submission.**

### **<ins>Installation Procedures</ins>**

As mentioned earlier,Round 2 of Dronetech'22 will be done in WeBots simulation and submissions in any other simulations will not be accepted!!

   #### **&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <ins>WeBots Simulator</ins>**
   
   - Webots is an open source and multi-platform desktop application used to simulate robots.
   - It provides a complete development environment to model, program and simulate robots.
   
   #### **&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <ins>Important Note</ins>**
   
   - The simulation of Dronetech'22 Round 2 is made and tested in WeBots-R2021a, which is last year version of WeBots(Latest version is R2022a).
   
   - **So it is highly recommended to the participants to use only this WeBots version(R2021a) for Dronetech'22 to avoid any type of error and clashes of depenedencies, versions.**
   
   &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; The Download link of WeBots-R2021a is given below:
   
   | OS                | File(s)                                                          |
   |:-----------------:|:-----------------------------------------------------------------:|
   | ![Logo](https://user-images.githubusercontent.com/2461619/60157660-95571180-97ef-11e9-8173-fa41a4092345.png)     | [webots-R2021a_setup.exe](https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a_setup.exe) |
   | ![Logo](https://user-images.githubusercontent.com/2461619/60157670-9be58900-97ef-11e9-9949-9bafc4f25226.png) | [webots_2021a_amd64.deb ](https://github.com/cyberbotics/webots/releases/download/R2021a/webots_2021a_amd64.deb)(Ubuntu 18.04 & 20.04)      <br> [webots-R2021a-x86-64.tar.bz2](https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a-x86-64.tar.bz2)(Ubuntu 20.04) <br>   [webots-R2021a-x86-64_ubuntu-18.04.tar.bz2](https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a-x86-64_ubuntu-18.04.tar.bz2)(Ubuntu 18.04)|
   | ![Logo](https://user-images.githubusercontent.com/2461619/60157679-a2740080-97ef-11e9-9cf0-30e3712d8fff.png)     | [webots-R2021a.dmg](https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a.dmg) |


### **<ins>Setup Requirements</ins>**
- Teams are supposed to use default python version in their OS.
- Modules and Libraries which they want to use should be installed on their default python.

### **<ins>Initialisation</ins>**
1. Download the folder named as "Dronetech'22" in your local machine.
2. Inside the worlds folder, launch the "Dronetech'22_arena" file.
<p align="center">
 <img src="/assets/image3.png">
 <p align="center">
 <i>Top view of the world without the ceiling</i><br> 
</p>

3. Search for Mavic2Pro in the left panel and select the controller as "my_controller.py" in the controllers/my_controller folder.
<p align="center">
 <img src="/assets/image1.png">
 <p align="center">
</p>

4. This my_controller.py file contains a boilerplate to help you with the code. You can use this or go with your own code.
5. This my_controller.py should contain all your code to complete the problem statement, and it should be flexible enough to work on both the arenas, be it the practice arena which is provided to you in this repo, or the final arena which will be revealed only at the time of final evaluation (avoid hardcoding).

# Scoring
- Let the marks obtained in the round 1 quiz be x out of 40..
- Let t be the total simulation time to complete the problem statement in the final arena.
- Then, total score of the team will be:
   
     **Score  =  (1000/𝑡)*(1 + 0.25*(𝑥/40))**
   
# Instructions regarding submissions
- **Round 1:** 
         <ul>
           <li>Team leader have to appear for the quiz on D2C on 9th march.</li>
           <li>Portal will remain open between 12 noon - 3 PM. Quiz can be given anytime between 12 noon - 3 PM and 40 minutes will be given after the start of the quiz.</li>
           <li>Only one submission per team will be accepted.</li>
           <li>Quiz link can be found [HERE](https://dare2compete.com/o/b61Yrn2?lb=JqIDm9e).</li>
         </ul>
         
- **Round 2:** 
         <ul>
           <li>Teams are needed to record the video of the drone completing the problem statement in the practice arena using any suitable screen recorder and have to upload it as an unlisted video on youtube.</li>
           <li>In the following google form, they are required to submit the youtube video link and "my_controller.py" file.</li>
           <li>Only one submission per team will be accepted. In case of multiple submissions, latest submission will be considered.</li>
           <li>Submit your link and file [here](https://forms.gle/7hngnc3S19aBeUGS7) by 13 March 2022 EOD.</li>
         </ul>
         
         
<div align="center">
<img src="/assets/LOGO2.png" alt="Technex logo" align="center" width="270">
</div> 
<div align="bottom">         
<img src= "/assets/technex.png" alt="technex logo" align="right" width="150">
</div>
