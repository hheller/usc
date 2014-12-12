 % Driving Force, Pedestrian interactions and boundary interactions %
% dv/dt = f0a + fab + faB %
%------------------------------------------------------------------------%

clear all
global m bdry p p1 p2 M vspur 
global tau r A1a A2a A3a A4a B1a B2a B3a B4a lambda A1 B1 VS VF1 VF2 VV1 VV2 
global dt v01 v02 vmax1 vmax2 L ml mr k Av Bv line

% Number of Pedestrians%
ml = input('Enter the number of Pedestrians, coming from left hand:');
mr = input('Enter the number of Pedestrians, coming from right hand:');
vl = input('Enter the number of Bikers, coming form left hand:');
vr = input('Enter the number of Bikers, coming from right hand:');
L=ml+vl;   %Anzahl von links kommender Personen
R=mr+vr;    %Anzahl von rechts kommender Personen
m=L+R;      %Die totale Anzahl Personen
% Timespan of simulation %
T = input('Enter the desired length of the simulation (in seconds): ');
line = input('Do you want a bikeline in your Simulation? Enter 1 for yes or 0 for no:');

% Select the geometry to be simulated %

%-----------------%
% model constants %
%-----------------%

% Relaxation time %
tau = 0.5;
% initial gewünschte Geschwindigkeit %
v01 = 2; %Gewünschte geschw der Fussgänger
v02 = 3.5*v01; %Gewünschte geschw der Veleofahrer
% Maximum desired velocity %
vmax1 = 1.3*v01;
vmax2 = 1.3*v02;
% Pedestrian radii %
r0 = 0.5;   %der Radius der Fussgänger
r1 = 2*r0;  %der Radius des Velofahrers
r = r0*ones(1,m);
%Der Radius der Velofahrer muss noch in r gelegt werden
for i=ml+1:L
    r(i)=r1;
end
for i=L+mr+1:m
    r(i)=r1;
end

% Pedestrian Interaction constants from (2.6.8) %
A1a = 0.02 ; %faktor
A2a = 0.5;
A3a = 0.0000035;
A4a = .4;%faktor
B1a = 5;%shape
B2a = 0.7;
B3a = 19.;
B4a = 1.4 ;% shape
lambda = 0.1;
%Anziehungskraft der Velofahrer
Av=3;%4
Bv=4;%5



k = 0.7;  %Die Dämpfung in y-Richtung (senkrecht zum Ziel), null=absolute Dämpfung, 1 = keine Dämpfung
% Boundary Interaction constants from (2.6.12) %
A1 = 5; %60
B1 = 0.1;
% Radius of Verlet Sphere %
VS = 4;
dt = 0.08; %Iterationszeit (in sekunden)
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

%Variabeln der Gl. zur Abstossungskraft der Velospur linie


VV1 = 0.01; %abstossungskraft der Linie Velofahrer (linear)
VV2= 0.3; %abstossungskraf der linie auf Velofahrer(im Exponent)
VF1 = 0.02; %abstossungskraf der Linie auf Fussgänger(linear)
VF2 = 0.3; %abstossungskraf der Linie auf Fussgänger (im Exponent)


%--------------------------------------%
% Initial position and velocity vector %
%--------------------------------------%



    
    Vx = zeros(1,m); %es starten alle mit geschwindigkeit 0 in x und y richtung
    Vy = zeros(1,m);
    
    if line == 1
    X = [25*rand(1,L)+25,25*rand(1,R)+50]; %Geändert, so dass alle von Anfang an im Spiel
    Y = [4.2*rand(1,ml)+9.3, 2*rand(1,vl)+6.4, 4.2*rand(1,mr)+9.3, 2*rand(1,vr)+6.4 ];%velofahrer starten in Velostreifen, fussgängeer im Fussgängersektor
    end
    if line == 0
         X = [25*rand(1,L)+25,25*rand(1,R)+50];
         Y = [6*rand(1,m)+7.5];
    end
        
    
    %Die Velofahrer starten im Schnitt um 25 weiter hinten als die
    %Fussgänger und sie sind etwas mehr verteilt
    for i=ml+1:L
        X(i)=30*rand;
    end
    for i=L+mr+1:m
        X(i)=30*rand+75;
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


start = u;
%----------------------------------------------------------------------%
% Desired destination - p is initially set as a waypoint (if required) %
%----------------------------------------------------------------------%
% p ist der



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
%options 1 setzt die optionen für's lösen der ode fest, zum bsp mit abstol
%die absolute toleranz und retol die relative
%options1 = odeset('AbsTol',1d-3,'RelTol',1d-4);
%[t1,u1] = ode45(@fun5,tspan,start,options1); %tspan = [0 T], start, 4m*1 wobei in den ersten 2m die startgeschwindigkeiten und in den 2ten 2m die startpositionen
[t1,O,unfalltotal, unfallorttotal]=Iteration_master(T,start);

%t1 beihnaltet einen spaltevektor mit allen Iterationszeitschritten
%O beinhaltet die Koordinaten von jeder Person zu jedem Zeitpunkt

pause
%Die Simulation gibt zwei Matrizen aus. t1 einfach alle zeitschritte übereinander
%u1 sind analog der starbedingungen eine 4*m matrix folgender struktur
%   ped links1 pedlinks2   pedrechts1 pedrechts2     pedlinks1     pedrechts1
%t1   vx  vy    vx   vy     vx   vy     vx  vy        X    Y        X     Y
%t2
%t3

%---------------------------------------%
% Plotting the paths of the pedestrians %
%---------------------------------------%

% Plotting the boundaries %
% Plottet zuerst bei festgehaltener zweiter Spalte alle x-Werte (laufen
% zweimal von 1-200, einmal immer punkt oben und einmal immer punkt unten)
% die Grenzen


    plot(bdry(:,1),bdry(:,2),'rx')
    axis([25,75,0,20]);
    hold on
    if line == 1
    plot(vspur(:,1),vspur(:,2),'r-')
    end
    hold on

% Plotting the pedestrian paths %
%die jeweiligen Pfade der Fussgänger sind abgelegt in einer 2x2 Matrix u,
%für jeden Zeitschritt einen x und y wert. es gibt
unfalltotal = unfalltotal/2
ufallorttotal = unfallorttotal
testanzahl = length(unfallorttotal); 

for i = 1:2:(2*m)
    if (i<=2*ml)
        plot(O(:,i),O(:,i+1),'b-')
        %axis([25,75,0,20])
        hold on
    elseif (i<=2*L)
        plot(O(:,i),O(:,i+1),'b*')
        hold on
    elseif (i<=2*(L+mr))
        plot(O(:,i),O(:,i+1),'r-')
        %axis([25,75,0,20])
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
        plot(bdry(:,1),bdry(:,2),'rx')
        hold on 
        plot(vspur(:,1),vspur(:,2),'r-')
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
        plot(bdry(:,1),bdry(:,2),'rx')
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
