
##Modeling the interaction of pedestrians and bikers
=====================================================


# MATLAB Fall 2014 – Research Plan

> * Group Name: Social Force Bikers
> * Group participants names: Sandro Giacomuzzi, Hannes Heller, Manuel Holzer
> * Project Title: Modeling the interaction of pedestrians and bikers


## General Introduction

As we started to discuss on different topics to find something we’d liked to model and simulate, we soon agreed on simulating a situation we are familiar with, e.g a phenomena we could observe our own. As we all are passionate bikers and we often ride through Zurich, where in many places pedestrians and bikers have to share the space: on sidewalks, on squares, on the lake promenade and at many sites more. We thought we could model the pedestrian- and biker flows at such sites and gain insight in how they interact and how capacity and security of such traffic ways could be improved. We are fascinated by the idea of simulating crowds by abstracting people as physical particles with the social force model and it’s valuable applications and are curious if and how it can be extended to other users of pedestrian roadways.
We want to observe how measurements as separating bikers and pedestrians by road marking or physical obstacles impact security and capacity as well as the impact of different parameters of both cyclists and walkers.
In the end, we hope the simulation shows similarities to the situation at the underpass of the Langstrasse on a Friday evening or the Zurich lake promenade at Bellvue on a sunny Sunday afternoon.

## The Model

We’ll take an existing model for simulating pedestrian flows; we will work with the social force model for pedestrian dynamics by Helbing and Molnar. In that model pedestrian flows are compared with fluids and gases.
Each pedestrian and each biker is represented by a Newtonian particle which experiences different forces: attraction to his personal spatial target, repulsion from other people and bikers or physical objects, preferred walking or driving speed, maybe attraction to a group of people or bikers he or she belongs to and so on. These forces, which are represented by vectors, result in a motion vector for every individual.
We will try to model the cyclists also as pedestrians but with other properties. For example their direction change ability will be lower because of their higher speed and inertia but they may have a bigger reaction-sphere due to their higher attentiveness. In order to quantify the security of a situation we may have to implement the scenario of an accident, e.g. particles collide with a certain force.


## Fundamental Questions

We want to find out how much separating sidewalks by road marking impacts capacity as well as safety at different capacity utilizations and have the model verified by our every day experience and eventually compare it to empirical data.
We also want to find out how changing parameters of attentiveness or perceptibility of both pedestrians and cyclists change flow and safety, which could simulate the use of for example a bike bell or illumination.


## Expected Results

We expect the cyclists to show a certain swarm behavior in order to maximize their speed. We expect ‚bike-lanes‘ for each direction to be formed amongst the pedestrians, regardless of their ‚physical existence‘. We also believe the separation of lanes to be unequally effective in terms of safety and performance depending on use to capacity.


## References

Helbing, Dirk; Molnár, Péter (1995). "Social force model for pedestrian dynamics"


## Research Methods

Agent-Based Model (?)


## Other

http://www.nzta.govt.nz/resources/research/reports/289/docs/289.pdf
http://paperity.org/p/4152322/an-agent-based-social-forces-model-for-driver-evacuation-behaviours
