Here are the things I want to talk about in this meeting

## What I did in robotics previously and how things here differ for what we are doing for colloboration
In robotics our rules for development looked like this:
1. Our master/main branch was protected. Whatever is on master is guaranteed to work (or have bugs that we know about 
but cannot do anything about them). Also, the master branch should always compile and have no runtime errors.
2. Never wrap anything in a try/catch (for the most part with some caveats). If we have to wrapped something in a try/catch, 
that means our code did something wrong. If we continue like nothing happened, most likely our code is in an invalid 
"state" which is dangerous for the robot and those operating it.
3. We wrote almost all of the software using state machines. The software was split into subsytems. Each subsystem ran 
independently and did not "know" about other parts of the software. Last year we had many subsystems, here is just a few 
   - Claw
   - Drivetrain (Swerve)
   - Wrist
   - Arm Rotation
   - Arm Extension
   - etc.

Last year, we needed interaction among the many subsystems (all the ones above and some more). Instead of having each 
subsystem communicate to each other, we had a super subsystem called the GrabAndGo. We treated it like an orchestrator. 
Each individual subsystem was like an instrument. They did not have to know about how what they did would impact the 
"entire piece." They just followed the orchestrator who knew what they "entire piece" is. The GrabAndGo controlled the 
interactions among the subsystem to automate the task of placing a game piece. The subsystem also didn't know all the positions 
they were supposed to go to. Instead, we had a GrabAndGoConstants class that held the vector position of the arm. Then 
when we had to go the position, we gave the Arm Rotation the theta position and the Arm Extension the magnitude. 
Let's run through a quick, simplified example of how a piece went from being on the ground to being placed:
   - The user tells the "picker" to come down which picks the game piece
   - Once the chamber senses the game piece, the GrabAndGo tells the Arm Rotation and Arm Extension subsystems to go into 
a position that grabs the game piece (and also tells the claw to grab the piece).
   - The GrabAndGo tells the Arm Rotation and Arm Extension subsystems to go into a "transport" position. The transport 
   position is the position that allowed the robot to drive and have the game piece secured. The arm was still inside the 
   robot, which kept the arm safe. There would be many opportunities for other robots to take off our arm if we drove around
   with it outside of our robot
   - Then when the user tells the robot to place the game piece, the GrabAndGo would tell the automatic placing subsystem 
   to get into position where the robot's arm can move to the position to place the piece. Then the arm moves to the placing 
   position. Then wait for the user to tell the robot to place.
   - The user can make adjustments to where the robot is to make placing the piece correctly. Then once the user tells the 
   robot to place the piece, the claw drops the game piece.

4. We always had two or more people that intimately knew each subsystem. We had high level documentation that everybody understood 
and everyone could see how the high level documentation related to the code. That way if somebody was gone, then we did 
not have a "knowledge lock" that prevented development on a particular subsystem because the person was gone.

The structure we used for robotics may not totally be possible because of how different this dynamic is than what I had 
in robotics. For the software team alone, we had ~20 people. Also we worked alot - like alot alot. I was told by multiple 
mentors that I was basically slacking off. At the time, I was doing 40+ hours into robotics a week. That was on top of 
school and applying to colleges. I got to the meetup place the shop at 7pm and left at 10pm on weekdays and Friday/Saturday 
I got there earlier and stayed later. Most people I knew stayed there until at least 1/2am. Since this is way more chill, 
which I am thankful for, I do not know how this will change the dynamics.

## What we want to do for spline generation
I do not know much about spline generation, but I have done some research on YouTube and some articles. I am currently 
thinking of doing a cubic spline. The spline is smooth enough and there is less computation that is needed to find them 
and therefore the running times are quicker. If we really wanted to, we could have predetermined splines that could be 
shifted over to optimize run times or make things easier to debug (doesn't really matter now, but just a thought). I 
wrote some math for a game engine I wrote and I thought it would be pretty easy to reuse that math. But before I get into 
that I'll show the resources I used to help with my knowledge of splines:

- https://pomax.github.io/bezierinfo/
- https://youtu.be/9R5UxtJQNNg?feature=shared
- https://youtu.be/pBtqaK0PzrA?feature=shared
- https://youtu.be/jvPPXbo87ds?feature=shared

I was thinking we could use a spline generation GUI, which should be helpful. You would be able to control as many point 
types as you want. For our robotics team we had three types: path controlling points, path action points, and required 
points. The path controlling points defined the path. The path action points defined the actions along the path. And 
the required points were points that the robot must go through (unlike path controlling points, which the robots may get 
close to, but is not guaranteed to pass through).

### Let's Get into the math and classes:
#### Polynomial
- Represented by a matrix of polynomial terms (a simple class that stores the degree and coefficient of each term)
- getIndefiniteIntegral() &rarr; returns a function which represents the indefinite integral
- getDerivative() &rarr; returns a function which represents the derivative
- getYValue(xCoordinate) &rarr; returns the y value at that x coordinate
- There would be many other helper methods, but that is the general idea

### Piecewise Function
- This is just a piecewise function in math. Depending where the x coordinate falls, a different function is used
- BoundedFunction[] &rarr; The math is done by a list of bounded function. A bounded function is a function that only 
returns a value if the x coordinate is within the domain [a, b]. There is the ability to do domain shifting with the input 
because it assumes that the function repeats. A bounded function is also guaranteed to be integratable and differentiable
- getIndefiniteIntegral() &rarr; returns a function which represents the indefinite integral
- getDerivative() &rarr; returns a function which represents the derivative
- getYValue(xCoordinate) &rarr; returns the y value at that x coordinate
- There would be many other helper methods, but that is the general idea

### Piecewise Followable Path
- A path that is followable. The path is defined by a piecewise function. And since the underlying piecewise function 
defines where the path goes, this class is super simple! It will keep track of the current time (and therefore path position).
It will use that time to perform calculations
- start() &rarr; activates the path
- run() &rarr; updates the current time
- getPosition() &rarr; the position the object should be at [x_coordinate, y_coordinate]
- getMotorSpeed()? &rarr; using the path knowledge, it will take a gander at what the motors speeds should be. The subsytem 
that knows the robot "intimately" would most likely do some sort of transformation based upon the robot's current movement 
vector.