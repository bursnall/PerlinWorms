# Perlin Worms!
Author: Treharne Bursnall with significant contributions by Dr. Jeffrey Paone

# Compilation
This program requires OpenGL and the GLFW, glew, and glm to run.

# Keys
Pressing Q/ESC will exit the program.  
Pressing M will switch between showing just the worm or the terrain around the worm.  
Pressing L will prompt the user for a new starting position for one of the worms.  
Pressing P will switch between allowing the worm to travel in three branching paths each segement or simply one path.  
Pressing -/+ will decrease/increase the threshold by 0.05f, to a minimum of 0.1f and maximum of 0.9f.  
Pressing 1/2 will decrease/increase the number of worms generated by 1, to a minimum of 1 and maximum of 6.  

# Notes
I don't allow there to be multiple worms in branching mode at once. When this option is chosen with multiple worms active, it will generate and draw the first in the list.
When adding more worms, the user is not allowed to chose their starting positions, they default to being rendered in the center of each face, hence a maximum of 6.
The current noise function is,,, interesting,,, so the worms may very well be one block long if the threshold is not decreased.

# Known Bugs
Alright, the big one.
Don't press a bunch of actively worm changing buttons in a row!! This for some reason causes the worms to get deleted. If the worms stop showing up, just rerun the project.
Re:above, I am currently trying to pass integers into a noise function. Since true integers in noise functions always return 0, I've been doing some convoluted multiplication to get around this fact. It works, somewhat. The worms will look funny and for some reason REALLY like the x direction.
Deleting and reloading the entirety of the buffers is not *optimal*. This will be adjusted in the near future.
