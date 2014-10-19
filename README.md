
##Modelling the interaction of pedestrians and bikers
=====================================================
	
As we started to discuss on different topics to find something, we’d liked to model and simulate, we soon agreed on choosing a subject we are linked to, e.g a phenomena we could observe our own. As we all are passionate bikers and we often drive through Zurich, where in many places pedestrians and bikers have to share the space: on pavements, on squares, on the lake promenade and at many sites more, we thought we could model the pedestrian- and biker flow at such sites.

With the model and simulation we are going to develop we would like to find out how to use the shared spaces. One more specific question would be: When does it make sense to have a line that separates the place in a pedestrian sector and in a bike sector. E.g. when there are only a very few bikers and a lot of walking people the pedestrians start to walk on the bike sector because there is a lot of free space and it gets more dangerous for all because bikers think they don’t have to watch out very much when there are in the bike sector where they in general drive faster compared to when there is no bike line. In contrary, when there are only a few pedestrian and many bikers, the faster bikers may leave the bike lane. We’d like to see these phenomena in our simulation and maybe make some conclusions on safety and capacity.


# MATLAB Fall 2014 – Research Plan

> * Group Name: Social Force Bikers
> * Group participants names: Sandro Giacomuzzi, Hannes Heller, Manuel Holzer
> * Project Title: Modelling the interaction of pedestrians and bikers


## General Introduction

As we started to discuss on different topics to find something we’d liked to model and simulate, we soon agreed on a subject we are familiar with, e.g a phenomena we could observe our own. As we all are passionate bikers and we often drive through Zurich, where in many places pedestrians and bikers have to share the space: on sidewalks, on squares, on the lake promenade and at many sites more. We thought we could model the pedestrian- and biker flows at such sites and gain insight in how they interact and how capacity and security of such traffic ways could be improved. We are fascinated by the idea of simulating crowds by abstracting people as physical particals with the social force models and it’s valuable applications and are curious how it can be extended to other users of pedestrian roadways.
In the end, we hope the simulation shows similarities to the situation at the unterpass of the Langstrasse on a Friday evening or the Zurich lake promenade at Bellvue on a sunny Sunday afternoon.

## The Model

We’ll take an existing model for simulating pedestrian flows; we will work with the social force model for pedestrian dynamics by Helbing and Molnar. In that model pedestrian flows are compared with fluids and gases.
Each pedestrian and in our model each biker is a Newtonian particle which experiences different forces: attraction to his personal spatial target, repulsion from other people and bikers, preferred walking or driving speed, maybe attraction to a group of people or bikers it belongs to, etc. These forces, which are represented by vectors, result in a motion vector for every individual. We will try to model the cyclists also as pedestrians but with other properties. For example their direction change ability will be lower because of their higher speed and inertia but they may have a bigger reaction-sphere due to their higher attentiveness. In order to quantify the security of a situation we may have to implement the scenario of an accident, e.g. if particles collide with a certain force.

## Fundamental Questions

We want to find out how much separating sidewalks by road marking improves capacity as well as safety at different capacity utilizations and have the model verified by our every day experience and eventually compare it to empirical data.
For instance, how do changing parameters of attentiveness or perceptibility of both pedestrians and cyclists change flow and safety?


## Expected Results

We expect the cyclists to show a certain swarm behavior in order to maximize their speed. We expect ‚bike-lanes‘ for each direction to be formed amongst the pedestrians. We also expect separation of lanes to be unequally effective in terms of safety and performance depending on use to capacity.


## References

Helbing, Dirk; Molnár, Péter (1995). "Social force model for pedestrian dynamics"


## Research Methods

Agent-Based Model (?)

## Other

http://www.nzta.govt.nz/resources/research/reports/289/docs/289.pdf
http://paperity.org/p/4152322/an-agent-based-social-forces-model-for-driver-evacuation-behaviours
