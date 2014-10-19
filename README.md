usc
===
Modelling the interaction of pedestrians and bikers
===================================================
	
As we started to discuss on different topics to find something, we’d liked to model and simulate, we soon agreed on choosing a subject we are linked to, e.g a phenomena we could observe our own. As we all are passionate bikers and we often drive through Zurich, where in many places pedestrians and bikers have to share the space: on pavements, on squares, on the lake promenade and at many sites more, we thought we could model the pedestrian- and biker flow at such sites.

With the model and simulation we are going to develop we would like to find out how to use the shared spaces. One more specific question would be: When does it make sense to have a line that separates the place in a pedestrian sector and in a bike sector. E.g. when there are only a very few bikers and a lot of walking people the pedestrians start to walk on the bike sector because there is a lot of free space and it gets more dangerous for all because bikers think they don’t have to watch out very much when there are in the bike sector where they in general drive faster compared to when there is no bike line. In contrary, when there are only a few pedestrian and many bikers, the faster bikers may leave the bike lane. We’d like to see these phenomena in our simulation and maybe make some conclusions on safety and capacity.

We’ll take an existing pedestrian model; probably we will work with the social force model for pedestrian dynamics by Helbing and Molnar. In that model pedestrian flows are compared with fluids and gases. 
Each pedestrian and in our model each biker is a Newton particle which experiences different forces: attraction to his personal spatial target, repulsion from other people and bikers, preferred walking or driving speed, maybe attraction to a group a person or biker belongs to, etc. These forces, which are represented by vectors, result in a motion vector for every individual.

We try to model the cyclists also as pedestrians but with other properties. For example their direction change ability will be lower due to their higher speed and inertia but they may have a bigger reaction-sphere due to their higher attentiveness. 
In the end, the simulation should be representative for the situation at the unterpass of the Langstrasse on a Friday evening or the Zurich lake promenade at Bellvue on a sunny Sunday afternoon.