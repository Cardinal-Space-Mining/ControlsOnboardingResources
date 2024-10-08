# The Knight's Tour
Borrowed from COMS 327 S24 taught by Jeremy Sheaffer
See this Wikipedia article about the Knight’s Tour problem: http://en.wikipedia.org/wiki/Knight%27s tour. 
Finding all directed, open tours on a standard, 8 × 8 chessboard is computationally intractable. 
To find them all on a 7 × 7 board in reasonable time would require a supercomputer. 
My quick-and-dirty solution finds all 6 × 6 solutions in about 2 hours and all 5 × 5 tours in under a second on a laptop. 
Write a C [OR C++] program to find all directed, open Knight’s Tours on a 5 × 5 chess board.
Assume the spaces of the board are numbered such as row 1 is 1-5 row 2 is 6-10 etc.

Print all tours as a list of numbers corresponding to the spaces in the order they are visited. 
For instance, here is one solution: 25,18,21,12,23,20,9,2,11,22,19,10,13,16,7,4,15,24,17,6,3,14,5,8,1. There are 1727 others. 
Your output should consist of 1728 lines, each line containing the numbers 1-25, each appearing exactly once, separated by commas (like the line above). 
This will allow for straightforward, automated checking of your output.


Extra Challenges (nothing below is required)
Generalize your program to handle boards of arbitrary dimension x X y.
Generalize your program to handle boards on a cylinder.
Generalize your program to handle boards on a torus.
Generalize your program to handle arbitrary boards (a graph with rectilinearly connected nodes).
Find only the subset of tours which are unique under rotation and reflection.
Print a board with the spaces numbered by the order in which they are visited.
Generate graphical representations of your tours. For instance, I used METAPOST to generate an image of the tour above and one on a full 8 X 8 board


I'm attaching two different solutions to this problem.  Neither of them makes any real effort at being fast, however, there is a huge performance difference between them.  
The "if" version searches for neighbors using conditionals, while the "offset" version iterates offsets of the current position.  
The "if" version is faster by a large margin because of reduced branching and better cache access patterns.
Here are the performance numbers for 6x6 tours:
tour_if: 124 minutes
tour_offset: 307 minutes
Speedup offset->if ~2.5x.  Same algorithm!  The only differences are the branches and the memory access pattern!
































