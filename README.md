== About ==

In this repository you will find implementations of various algorithms I have studied as part of my doctoral research into path-finding. This includes Jump Point Search and Rectangular Symmetry Reduction.

If you would like more information about my work, or are looking for copies of published papers, please visit my [http://users.rsise.anu.edu.au/~dharabor ANU homepage]. 

The code hosted here is written in C++. Much of it is built from the ground up. Some is built on top of Nathan Sturtevant's open-source pathfinding library [http://hog2.googlecode.com Hierarchical Open Graph]. All of it is made available under the terms of the GNU GPL Version 2. 

== Releases == 
The latest release of Jump Point Search is `r539`. This version includes an implementation of the original algorithm (published in 2011) and a number of subsequent improvements (published in 2014). You can download all of these sources by issuing the following command:

{{{ 
svn checkout https://ddh.googlecode.com/svn/tags/warthog-r539/ jps
}}}

The code is best compiled with GCC 4.8.1. I have tested it on OSX 10.8.4 and Ubuntu 13.10.
I make it available in the interest of scientific research and without any guarantees.