clear all
global m bdry p p1 p2 M vspur A5
global tau r A1a A2a A3a A4a B1a B2a B3a B4a lambda A1 B1 VS VF1 VF2 VV1 VV2 
global dt v01 v02 vmax1 vmax2 L ml mr k Av Bv line

% Number of pedestrians%
ml = input('Enter the number of Pedestrians, coming from left hand:');
mr = input('Enter the number of Pedestrians, coming from right hand:');
% Numbers of bikers%
vl = input('Enter the number of Bikers, coming form left hand:');
vr = input('Enter the number of Bikers, coming from right hand:');
L=ml+vl;   %# person left
R=mr+vr;   %# person right
m=L+R;     %# total amount of peoplen
% Timespan of simulation %
T = input('Enter the desired length of the simulation (in seconds): ');
line = input('Do you want a bikeline in your Simulation? Enter 1 for yes or 0 for no:');

% Select the geometry to be simulated %

%-----------------%
% model constants %
%-----------------%

% Relaxation time %
tau = 0.5;
% initial desired velocity %
v01 = 2; %Gewünschte geschw der Fussgänger
v02 = 3*v01; %Gewünschte geschw der Veleofahrer
% Maximum desired velocity %
vmax1 = 1.3*v01;
vmax2 = 1.3*v02;
% Pedestrian radii %
r0 = 0.3;   %der Radius der Fussgänger
r1 = 1.5*r0;  %der Radius des Velofahrers
r = r0*ones(1,m);
% matrix with radii %
for i=ml+1:L
    r(i)=r1;
end
for i=L+mr+1:m
    r(i)=r1;
end


%  Interaction constants %
A1a = 1.5 ; %faktor
A2a = 2;
A3a = 5 ;
A4a = 2;%faktor
B1a = 1;%shape
B2a = 0.3;
B3a = 8;
B4a = 0.2;% shape
lambda = 0.3;

%Attractive force slipstream %
Av=16;
Bv=4;

%Ausweichkraft bei direkt
A5=10;

% damping %
k = 0.4;  %Die Dämpfung in y-Richtung (senkrecht zum Ziel), null=absolute Dämpfung, 1 = keine Dämpfung
% Boundary Interaction constants  %
% same for peds. and bikes %
A1 = 5; %60
B1 = 0.1;
% Radius of Verlet Sphere %
VS = 4;
%timestep (in s)
dt = 0.06; %Iterationszeit (in sekunden)
M0 = 1;     %Masse der Fussgänger
M1 = 2*M0;   %Masse der Velofahrer
M = ones(1,m);   %der Massenvektor
%Die mMasse der Velofahrer muss noch in M gelegt werden
for i=ml+1:L
    M(i)=M1;
end
for i=L+mr+1:m
    M(i)=M1;
end

% variables for attraction/repulsion separation line %


VV1 = 2; %abstossungskraft der Linie Velofahrer (linear)
VV2= 0.3; %abstossungskraf der linie auf Velofahrer(im Exponent)
VF1 = 1.5; %abstossungskraf der Linie auf Fussgänger(linear)
VF2 = 0.3; %abstossungskraf der Linie auf Fussgänger (im Exponent)


%--------------------------------------%
% Initial position and velocity vector %
%--------------------------------------%



 % all starting at speed zero %
    Vx = zeros(1,m); %es starten alle mit geschwindigkeit 0 in x und y richtung
    Vy = zeros(1,m);
 %starting constellation with line
    if line == 1
    X = [25*rand(1,L)+25,25*rand(1,R)+50]; % alle von Anfang an im Spiel
    Y = [5.5*rand(1,ml)+8, 5*rand(1,vl)+6.3, 5.5*rand(1,mr)+8, 5*rand(1,vr)+6.3 ];%velofahrer starten in Velostreifen, fussgängeer im Fussgängersektor
    end
    
    %starting constellation without line
    if line == 0
         X = [25*rand(1,L)+25,25*rand(1,R)+50];
         Y = [7.4*rand(1,m)+6.3];
    end
        
    
    %Die Velofahrer starten im Schnitt um 25 weiter hinten als die
    %Fussgänger und sie sind etwas mehr verteilt
    for i=ml+1:L
        X(i)=40*rand;
    end
    for i=L+mr+1:m
        X(i)=40*rand+75;
    end


% Setting u - the initial position and velocity vector %

for i = 1:m %2m*1 Matrix, mit den Elementen der beiden Geschwindigkeiten in x-und y-Richtung hintereinander (zuerst x, dann y)
    u((2*i)-1) = Vx(i);
    u((2*i)) = Vy(i);
end
for i = 1:m %u wird erweritert zu einem 4m*1-Vektor wobei in den 2m-neuen Elementen die X- und Y-Werte abgespreichert werden (zuerst X dann Y)
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
    %p1 ist das zielarray der linksstartenden, ausserhalb des sichtbaren
    %bereichs ganz rechts, p2 für die rechtsstartenden und p wird erstmal leer
    %definiert und dann wird in der fun5 funktion jedem fussgänger entweder
    %das eine oder andere zugeordnet
    p1 = [[100*ones(1,(6/0.1)+1)]',[7:0.1:13]'];    %X-Koordinaten sind 100, und Y-koordinaten sind in 61 punkte unterteilt im Bereich der Boundary
    p2 = [[zeros(1,(6/0.1)+1)]',[7:0.1:13]'];        %X-Koordinaten sind null, und Y-koordinaten sind in 61 punkte unterteilt im Bereich der Boundary

%----------------%
% Boundary Array %
%----------------%


    %Das für uns wichtige Boundary Array erstellt einfach eine Matrix 2x201
    %Matrix und setzt im 2D Raum eine obere und eine Untere Grenze, jeweils
    %auf höhe 1 und auf höhe 6. geplottet wird das ganze dann mit 'rx' kleinen
    %kreuzchen. Die erste Spalte ist die x-Achse, die zweite die Y-Achse

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

pause


%---------------------------------------%
% Plotting the paths of the pedestrians %
%---------------------------------------%

% Plotting the boundaries %



    plot(bdry(:,1),bdry(:,2),'kx')
    axis([25,75,0,20]);
    hold on
    if line == 1
    plot(vspur(:,1),vspur(:,2),'g-')
    end
    hold on

% counting the dangerous situations %
unfalltotal = unfalltotal/2 + mod(unfalltotal/2,1)
ufallorttotal = unfallorttotal
testanzahl = length(unfallorttotal); 
% Plotting the pedestrian paths %

for i = 1:2:(2*m)
    if (i<=2*ml)
        plot(O(:,i),O(:,i+1),'b-')
        
        hold on
    elseif (i<=2*L)
        plot(O(:,i),O(:,i+1),'b*')
        hold on
    elseif (i<=2*(L+mr))
        plot(O(:,i),O(:,i+1),'r-')
        
        hold on
    else
        plot(O(:,i),O(:,i+1),'r*')
        hold on
    end
end
pause
%-----------------------------------------------------------%
% Making the Movie - plots the pedestrian positions at each %
% timestep and then uses each plot as a movie frame %
%-----------------------------------------------------------%
 if line == 1
for j = 1:length(t1)
    % Plotting the boundaries %
        hold off
        plot(bdry(:,1),bdry(:,2),'kx')
        hold on 
        plot(vspur(:,1),vspur(:,2),'g-')
        axis([25,75,0,20]);
        hold on 
    
    % Plotting the pedestrians %
   
    for i = 1:2:2*m
        if (i<=2*ml)
            plot(O(j,i),O(j,i+1),'bo')
            axis([25,75,0,20])
            hold on
        elseif (i<=2*L)
            plot(O(j,i),O(j,i+1),'b*')
            axis([25,75,0,20])
            hold on
        elseif (i<=2*(L+mr))
            plot(O(j,i),O(j,i+1),'ro')
            axis([25,75,0,20])
            hold on
        else
            plot(O(j,i),O(j,i+1),'r*')
            axis([25,75,0,20])
            hold on
        end
    end
    F(j) = getframe;
end
 end
 
 if line == 0
     for j = 1:length(t1)
    % Plotting the boundaries %
        hold off
        plot(bdry(:,1),bdry(:,2),'kx')
        axis([25,75,0,20]);
        hold on 
    
    % Plotting the pedestrians %
   
    for i = 1:2:2*m
        if (i<=2*ml)
            plot(O(j,i),O(j,i+1),'bo')
            axis([25,75,0,20])
            hold on
        elseif (i<=2*L)
            plot(O(j,i),O(j,i+1),'b*')
            axis([25,75,0,20])
            hold on
        elseif (i<=2*(L+mr))
            plot(O(j,i),O(j,i+1),'ro')
            axis([25,75,0,20])
            hold on
        else
            plot(O(j,i),O(j,i+1),'r*')
            axis([25,75,0,20])
            hold on
        end
    end
    F(j) = getframe;
end
 end
 

clf %clf deletes from the current figure all graphics objects whose handles are not hidden (i.e., their HandleVisibility property is set to on).