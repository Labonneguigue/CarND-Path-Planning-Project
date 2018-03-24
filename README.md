[uml]: ./docs/diagrams/architecture.png
[simulator]: ./docs/images/simulator.tiff

# Path Planning on a 3 Lanes Highway

Self-Driving Car Engineer Nanodegree Program

## Project Environment

### Simulator

Like previous projects, in order to test our code Udacity has provided us another great simulated environment where I can test my code.

![alt text][simulator]

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals

Here are the goals as they were stated to complete this project :

> In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Provided Data

#### Highway Map

Udacity provides waypoints along the highway. Each of them contains [x,y,s,dx,dy] values.
* x and y are the waypoint's map coordinate position
* the s value is the distance along the road to get to that waypoint in meters,
* the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Assuming that the mapping companies like HERE succeed at their task, Self Driving Cars will indeed have access to high resolution maps accurate down to the centimeter level. In that prospect the noiseless data I was given for the map seems reasonable.

#### My Autonomous Vehicle Localization Data

The noiseless data provided by the simulator regarding my car is the following :

* `["x"]` : The car's x position in map coordinates
* `["y"]` : The car's y position in map coordinates
* `["s"]` : The car's s position in frenet coordinates
* `["d"]` : The car's d position in frenet coordinates
* `["yaw"]` : The car's yaw angle in the map
* `["speed"]` : The car's speed in MPH

Note: Throughout the code I've called my vehicle `MyAV` to differentiate it from the other vehicles.

#### Surrounding Car's Data

* `["sensor_fusion"]` : A 2d vector of cars and then that car's :
    * car's unique ID
    * car's x position in map coordinates
    * car's y position in map coordinates
    * car's x velocity in m/s
    * car's y velocity in m/s
    * car's s position in frenet coordinates
    * car's d position in frenet coordinates.

That data only contains data about cars on my side of the road.


---


## Algorithm

### Design

Here is my code architecture that I used to answer this problem. There still too many inter-dependencies that I'd like to remove. Ideally, the Path Planner should be the only instance to have knowledge of every modules and each module (Predictor, Trajectory Generator, Sensor Fusion, etc ..) being agnostic of the other.

![alt text][uml]


### Modules

#### Behavior Planner

The behavior planner is responsible for issuing general driving decisions like _Stay in Lane_, _Turn Right_ or _Turn Left_.

How it works is by aiming to reach a point on the map far ahead and compute the cost of (read "time to reach that point") changing/staying in each lane. That cost function uses data about the other cars on the road provided by the Sensor Fusion module to decide which lane is the best fit. It uses the speed of each lane and computes the time to reach a far target at the speed of each lane.

Since the Behavior Planner provides high level decisions that are not always subject to rapid change I reduced its frequency to be ran once in every 20 calls. However I can be woken up by the Predictor if necessary and be ran every frames whenever it is deemed to do so.

#### Trajectory Generator

**1. Major Waypoints**

First, given the lane I'm targeting to end up in, I generate 3 major path waypoints 50 meters away from each other.
I also create 2 points very close to my position which reflect the current orientation of my car. More on why I did this bellow...

**2. Spline Generation**

To know how I'm going to reach one point after another, I create a **spline** which runs through these major waypoints. A spline is a polynomial regression that goes exactly through each and every points that are given to it. Since I create these 2 points where my car is at, the generated spline will be tangent to my current trajectory and won't create too much jerk or acceleration.

**3. Subsampling**

Since the simulator expects me to provide him the position of where I want my car to every 0.02 seconds, I must subsample intermediate points onto the generated spline. I do so knowing that the distance between the sample will determine my speed.

**4. Reuse of previous Trajectory**

Lastly, since I provide to the simulator more points than are executed at each steps, some waypoints remain. To save computation time, I reuse the previous points and only top the trajectory up every time. The trick is to keep a representation of my car (position & heading) of how it would be if I reached the end of the trajectory in order to generate the next waypoints in a smooth manner.

Special thanks to the [Spline library](http://kluge.in-chemnitz.de/opensource/spline/) that was really useful to generate smooth trajectories! 👍

##### Speed Control : PID Controller 🛂

Let's say at some point a car change lane in front on me without keeping what I consider to be a safe distance between us, then I need to slow down in order to establish that distance. To smooth out this process, I added a PID controller which outputs a speed based on the error in distance in-between myself and the car ahead relative to a safe distance that I hardcoded.

```cpp
double pidCorrectedSpeed = 0.0;
if (distanceCarAhead < policy::safeDistance * 1.5)
{
    // I compute the Distance Error as the delta between the optimal
    // safe distance and the actual distance between us
    double deltaDistance = policy::safeDistance - distanceCarAhead;
    mSpeedRegulator.updateError(deltaDistance);
    pidCorrectedSpeed = mSpeedRegulator.totalError();
}

double appliedSpeed = mCurrentTargetVelocityMs + pidCorrectedSpeed;
```

```cpp
constexpr static const int policy::safeDistance = 10; ///< Safe distance to always (try to) keep between cars.
```

#### Predictor

My Predictor module is providing information about the surrounding vehicles. It helps the Trajectory Generator and the Behavior Planner to have access to the data in a more intelligible manner.
Every iteration, it raises warnings if necessary:

```c++
void Predictor::environmentalEvaluation(Warnings& warnings) const;
```

The 2 types of warnings are:
* Slow car ahead in my current lane
* A car is changing lane onto mine.

If any warning is raised, the Behavior Planner will be called and it would be its responsibility to assess the full situation and decide what to do.

#### Modular Design

##### Highway Model

I designed my algorithm in a modular fashion with the least hardcoded values as possible and as few dependencies as possible. Following this concept, the Behavior Planner module does have to know how many lanes there are on the road. The `Highway` structure keeps track of how many lanes there are on the highway and provides the available lanes number to whoever wants them.

```cpp
constexpr static const int defaultNbLanes = 3; ///< Default number of lanes on the road at startup time
```

You'll think it doesn't make much sense in that project but in real life, an AV would likely leave that 3 lanes highway scenario and would need to update its internal representation of the road it is currently on.

```c++
/** Changes dynamically the size of the road to adapt to another
 *  size of highway
 *
 * @param[in] nbLanes Number of lanes that contains this highway
 *
 */
void setNumberLanes(const int nbLanes);
```

##### Driving Policy

I added a document `policy.h` collecting many hardcoded values which influence how the car drive.
In Europe cars are required to keep right when they are not overtaking whereas in the US, the driving policy is more lenient and do not enforce any behavior. I added a boolean flag to enable both driving styles.

```c++
constexpr static const bool keepRightLane = false; ///< False: US, True: EU
```

For now this is a compile time setting but it could easily be modifiable at runtime if the car detected that it crossed the Atlantic. 🌊

---

## Wanna try for yourself ?

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

#### Create Xcode Project

1. Create a xcode folder in the build folder: `mkdir xcode && cd xcode`
2. Create the xcode project using cmake: `cmake ../.. -G "Xcode"`
3. Open the project: `open Path_Planning.xcodeproj`
Because Xcode creates the executable binary in a folder and we need to open the map data contained in a csv file, I have set a preprocessor directive to change the hardcoded path to it. The following build flag enables it:
4. Click on the blue Path_Planning project and under xxx add the following flag to the file main.cpp: `-DXCODE`

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
* libuv 
* openssl

### Third Party Libraries

* [Spline library](http://kluge.in-chemnitz.de/opensource/spline/)
