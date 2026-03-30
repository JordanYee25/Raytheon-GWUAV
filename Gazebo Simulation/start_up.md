# :dizzy: Navigation Simulations :dizzy:
<p>Navigation simulations with SITL, MAVProxy, Gazebo, MAVLINK, DroneKit, and pymavlink.</p>

## Shell script simstart.sh
<p> Running, just the shell script in the terminal should start up SITL with MAVProxy, Gazebo virtual world, and the autonomous mission scripts. You can specify different scripts from the Auto Scipts folder. </p>

## How to use capstone.world

### Explanation

<p>Capstone.world is a Gazebo simulation environment. It is supposed to emulate the real settings of the drone flight. You can make this as specific and realistic as you want by defining the gravity, time, wind conditions, etc. Gravity is default 9.8 m/s^2,
but think about simulations on the moon. Specifically for capstone.world the time is defined as early morning with clouds and natural lighting. Morning because the competition starts then and clouds because of a typical cloudy day at Northern Virginia. <br>

The .world file uses XML and SDF standards. Models were used to include drones and ArUco markers. <br> 

Also, the ArUco markers positions can be changed by editing the *<pose> </pose>* first two points which are x,y coordinates. This will give random locations for later tests with the drone. <br>

Now, other simulations and programs can be written to make the drone model fly within this .world (think of .launch scripts). Run this command in one terminal:</p>
```
gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world
```
<p>In the other terminal, you will run the .launch script.</p>

![capstone world_directory](https://github.com/user-attachments/assets/2255394d-d981-472e-938d-cdabfe288d9c)


![capstone world_sim](https://github.com/user-attachments/assets/65682ce2-08d3-47d4-a100-cc2029a0106d)
