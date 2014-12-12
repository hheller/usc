% Driving Force, Pedestrian interactions and boundary interactions %
% dv/dt = f0a + fab + faB %
%------------------------------------------------------------------------%

function [unfalltotal,unfallorttotal,Z] = social_force_master_auswerter(fl,fr,vel,ver,linie, dauer)
global m bdry p p1 p2 M vspur A5
global tau r A1a A2a A3a A4a B1a B2a B3a B4a lambda A1 B1 VS VF1 VF2 VV1 VV2 
global dt v01 v02 vmax1 vmax2 L ml mr k Av Bv line

% Number of Pedestrians%
ml = fl; %input('Enter the number of Pedestrians, coming from left hand:');
mr = fr; %input('Enter the number of Pedestrians, coming from right hand:');
vl = vel; %input('Enter the number of Bikers, coming form left hand:');
vr = ver; %('Enter the number of Bikers, coming from right hand:');
L=ml+vl;   %Anzahl von links kommender Personen
R=mr+vr;    %Anzahl von rechts kommender Personen
m=L+R;      %Die totale Anzahl Personen
% Timespan of simulation %
T = dauer; %input('Enter the desired length of the simulation (in seconds): ');
line = linie; %input('Do you want a bikeline in your Simulation? Enter 1 for yes or 0 for no:');

% Timespan of simulation %
% T = input('Enter the desired length of the simulation (in seconds): ');
% line = input('Do you want a bikeline in your Simulation? Enter 1 for yes or 0 for no:');


%-----------------%
% model constants %
%-----------------%

% Relaxation time %
tau = 0.5;
% initial desired velocity %
v01 = 2; %of peds.
v02 = 3*v01; %of bikes
% Maximum desired velocity %
vmax1 = 1.3*v01;
vmax2 = 1.3*v02;
% Pedestrian radii %
r0 = 0.3;   %peds.
r1 = 1.5*r0;  %bikes
r = r0*ones(1,m);
% matrix with radii %
for i=ml+1:L
    r(i)=r1;
end
for i=L+mr+1:m
    r(i)=r1;
end


%  Interaction constants %
A1a = 1.5 ; %faktor % ped.
A2a = 2;          % ped.
A3a = 5 ;           % bike
A4a = 2;%faktor   % bike
B1a = 1;%shape      % ped.
B2a = 0.3;          % ped
B3a = 8;           % bike
B4a = 0.2;% shape   % bike 
lambda = 0.3;

%Attractive force slipstream %
Av=16;
Bv=4;

% ausweichkraft parameter

A5 = 10; 


% damping %
k = 0.4;  %damping in y-direction (perpendicular to destination)
          %null=absolute damping, 1=no damping
          
% Boundary Interaction constants  %
% same for peds. and bikes %
A1 = 5; 
B1 = 0.1;

% Radius of Verlet Sphere %
VS = 4;

%timestep (in s)
dt = 0.06; 
M0 = 1;     % mass of peds
M1 = 2*M0;  % mass of bikes

% massvector %
M = ones(1,m);  
for i=ml+1:L
    M(i)=M1;
end
for i=L+mr+1:m
    M(i)=M1;
end

% variables for attraction/repulsion separation line %


VV1 = 2; %attraction constant bike (linear)
VV2= 0.3; %attraction constant bike (exponent)
VF1 = 1.5; %repulsion constant ped(linear)
VF2 = 0.3; %repulsion constant ped (exponent)


%--------------------------------------%
% Initial position and velocity vector %
%--------------------------------------%

% all starting at speed zero %
    Vx = zeros(1,m); 
    Vy = zeros(1,m);
    
    %starting constellation with line
    if line == 1
    X = [25*rand(1,L)+25,25*rand(1,R)+50]; 
    %bikes starting in lane, peds outside %
    Y = [5.5*rand(1,ml)+8, 5*rand(1,vl)+6.3, 5.5*rand(1,mr)+8, 5*rand(1,vr)+6.3 ];
    end
    
    %starting constellation without line
    if line == 0
         X = [25*rand(1,L)+25,25*rand(1,R)+50];
         Y = [7.4*rand(1,m)+6.3];
    end
    
    for i=ml+1:L
        X(i)=40*rand; 
    end
    for i=L+mr+1:m
        X(i)=40*rand+75;
    end


% Setting u - the initial position and velocity vector %

for i = 1:m %2m*1 matrix with speed component x, then y for each individual
    u((2*i)-1) = Vx(i);
    u((2*i)) = Vy(i);
end
for i = 1:m %u extended to 4m*1-vektor with positions coming after speed, first X then Y values
    u((2*m)+(2*i)-1) = X(i);
    u((2*m)+(2*i)) = Y(i);
end
   for i= 1:m;
    u(4*m+i) = 0;
    end

% saving start config %
start = u;


%----------------------------------------------------------------------%
% Desired destination - p is initially set as a waypoint (if required) %
%----------------------------------------------------------------------%
    p = zeros(m,2);
    %destination arrays, p1 for people from left, p2 right
    
    p1 = [[100*ones(1,(6/0.1)+1)]',[7:0.1:13]'];   
    p2 = [[zeros(1,(6/0.1)+1)]',[7:0.1:13]'];      

%----------------%
% Boundary Array %
%----------------%

    bdry = [    [-50:0.1:150   ,    -50:0.1:150]'...
        ,[6*ones(1,(200/0.1)+1)    ,    14*ones(1,(200/0.1)+1)]'];
    
     %--------------%
     %   Velospur   %       
     %--------------%

     vspur = [[-50:0.1:150]',[9*ones(1,(200/0.1)+1)]']; % raumilche Anordnun
 


%----------------------------------------%
% The solver for the system - here ode45 %
%----------------------------------------%

[t1,O,unfalltotal, unfallorttotal, Z]=Iteration_master(T,start);

% t1 time step vector
%O coordinates of each person at each timestep

unfalltotal = unfalltotal/2;
ufallorttotal = unfallorttotal;
testanzahl = length(unfallorttotal); 
Z = Z;

% pause
% 
% 
% %---------------------------------------%
% % Plotting the paths of the pedestrians %
% %---------------------------------------%
% 
% % Plotting the boundaries %
% 
%     plot(bdry(:,1),bdry(:,2),'rx')
%     axis([25,75,0,20]);
%     hold on
%     if line == 1
%     plot(vspur(:,1),vspur(:,2),'r-')
%     end
%     hold on
% 
% 
% 
% % counting the dangerous situations %
% unfalltotal = unfalltotal/2 + mod(unfalltotal/2,1);
% ufallorttotal = unfallorttotal;
% testanzahl = length(unfallorttotal); 
% 
% % Plotting the pedestrian paths %
% 
% for i = 1:2:(2*m)
%     if (i<=2*ml)
%         plot(O(:,i),O(:,i+1),'b-')
%         %axis([25,75,0,20])
%         hold on
%     elseif (i<=2*L)
%         plot(O(:,i),O(:,i+1),'b*')
%         hold on
%     elseif (i<=2*(L+mr))
%         plot(O(:,i),O(:,i+1),'r-')
%         %axis([25,75,0,20])
%         hold on
%     else
%         plot(O(:,i),O(:,i+1),'r*')
%         hold on
%     end
% end
% pause
% 
% %-----------------------------------------------------------%
% % Making the Movie - plots the pedestrian positions at each %
% % timestep and then uses each plot as a movie frame %
% %-----------------------------------------------------------%
% 
%  if line == 1
% for j = 1:length(t1)
%     % Plotting the boundaries %
%         hold off
%         plot(bdry(:,1),bdry(:,2),'rx')
%         hold on 
%         plot(vspur(:,1),vspur(:,2),'r-')
%         axis([25,75,0,20]);
%         hold on 
%     
%     % Plotting the pedestrians %
%    
%     for i = 1:2:2*m
%         if (i<=2*ml)
%             plot(O(j,i),O(j,i+1),'bo')
%             axis([25,75,0,20])
%             hold on
%         elseif (i<=2*L)
%             plot(O(j,i),O(j,i+1),'b*')
%             axis([25,75,0,20])
%             hold on
%         elseif (i<=2*(L+mr))
%             plot(O(j,i),O(j,i+1),'ro')
%             axis([25,75,0,20])
%             hold on
%         else
%             plot(O(j,i),O(j,i+1),'r*')
%             axis([25,75,0,20])
%             hold on
%         end
%     end
%     F(j) = getframe;
% end
%  end
%  
%  if line == 0
%      for j = 1:length(t1)
%     % Plotting the boundaries %
%         hold off
%         plot(bdry(:,1),bdry(:,2),'rx')
%         axis([25,75,0,20]);
%         hold on 
%     
%     % Plotting the pedestrians %
%    
%     for i = 1:2:2*m
%         if (i<=2*ml)
%             plot(O(j,i),O(j,i+1),'bo')
%             axis([25,75,0,20])
%             hold on
%         elseif (i<=2*L)
%             plot(O(j,i),O(j,i+1),'b*')
%             axis([25,75,0,20])
%             hold on
%         elseif (i<=2*(L+mr))
%             plot(O(j,i),O(j,i+1),'ro')
%             axis([25,75,0,20])
%             hold on
%         else
%             plot(O(j,i),O(j,i+1),'r*')
%             axis([25,75,0,20])
%             hold on
%         end
%     end
%     F(j) = getframe;
% end
%  end
end

%clf %clf deletes from the current figure all graphics objects whose handles are not hidden (i.e., their HandleVisibility property is set to on).