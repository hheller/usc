% Driving Force, Pedestrian interactions and boundary interactions %
% dv/dt = f0a + fab + faB %
%------------------------------------------------------------------------%
% beschl=fziel+fpersonen+fboundaries
clear all
global m mb bdry p p1 p2 D way BDRY
global tau taub v0a0 v0a0b vmax vmaxb r rb A1a A1ab A2a A2ab B1a B1ab B2a ...
    B2ab lambda lambdab A1 A1b B1 B1b VS VSb dist distb
%tau beschreibt den wunsch einer pers wieder auf ihre gewünschte geschw zu
%beschleunigen (1/tau*differenz v ist gleich f0a)
%dist ist abstand ab welchem ziel erreicht

% Number of Pedestrians%
m = input('Enter the number of Pedestrians in the system (must be even):');
% Numer of bikers
mb = input('Enter the numer of bikers in the system (must be even):');
% Timespan of simulation %
T = input('Enter the desired length of the simulation (in seconds): ');
tspan = [0 T];
% Select the geometry to be simulated %
K=2;
%----------------------------------------------------------%
% Vector used in waypoint calculation - their value tells %
% pedestrian alpha which waypoint to head for %
%----------------------------------------------------------%
D = zeros(1,m+mb); %b für die dazukommenden bikers
%-----------------%
% model constants %
%-----------------%

% Relaxation time %
tau = 0.5;
% BIKER %
taub = 0.7;

% Initial desired velocity %
v0a0 = 1.34;
% BIKER %
v0a0b = 6.6;

% Maximum desired velocity %
vmax = 1.3*v0a0;
% Biker%
vmaxb = 1.5*v0a0b;

r= ones(1,m+mb);
% Pedestrian radii %
%r = 0.3*ones(1,m);
% Biker %
%rb = 1.2*ones(1,mb);
r(1,1:m) = 0.3;
r(1,(m+1):mb) = 1.2;



% Pedestrian Interaction constants from (2.6.8) %
A1a = 2;        %
A2a = 2;        %
B1a = 0.3;      %
B2a = 0.2;      %
lambda = 0.75; % Form der Reaktionsphäre
% Bikers %
A1ab = 3;
A2ab = 2.5;
B1ab = 0.15;
B2ab = 0.4;
lambdab = 0.5;

% Boundary Interaction constants from (2.6.12) %
A1 = 5; 
B1 = 0.1;
% Biker %
A1b = 7;
B1b = 0.09;

% Radius of Verlet Sphere %
VS = 10;
% Biker %
VSb = 25;

% Distance to waypoint at which it deem to have been reached %
dist = 1.2;
% Biker %
distb = 2;
%--------------------------------------%
% Initial position and velocity vector %
%--------------------------------------%

if (K == 1)
    way = 1;
    Vx = [1.34*rand(1,m/2), -1.34*rand(1,m/2),v0a0b*rand(1,mb/2),-v0a0b*rand(1,mb/2)]; %initiale Zufallsgeschwindigkeit zwischen 0 und 1.34 in x-richtung (negativ bzw. positiv Anzahl m/2)
    Vy = zeros(1,m+mb); %initialle geschwindigkeit senkrecht zu x ist null für alle m
    X = [9*rand(1,m/2),9*rand(1,mb/2),9*rand(1,m/2)+11,9*rand(1,mb/2)+11]; %Zufallsvektor, wobie die erste Hälfte zwischen 0-9 und die zweite zwischen 11-20
    Y = [2*rand(1,(m+mb))+3.5]; %Zufallsvektor, welcher für alle m zwischen 3.5-5.5
elseif (K == 2)
    way = 0;
    Vx = zeros(1,m+mb); %es starten alle mit geschwindigkeit 0 in x und y richtung
    Vy = zeros(1,m+mb);
    X = [25*rand(1,m/2)+25 , 25*rand(1,m/2)+50, 25*rand(1,mb/2)+25, 25*rand(1,mb/2)+50]; %eine m langezeile, zuerst die m/2 linken, dann die rechten. Geändert, so dass alle von Anfang an im Spiel
    Y = [6*rand(1,m+mb)+7.5];
elseif (K == 3)
    way = 1;
    Vx = zeros(1,m);
    Vy = zeros(1,m);
    X = [5*rand(1,m/2),5*rand(1,m/2)+15];
    Y = [10*rand(1,m)];
elseif (K == 4)
    Vx = zeros(1,m);
    Vy = zeros(1,m);
    X = [45*rand(1,m/2)-40,45*rand(1,m/2)+35];
    for i = 1:m
        if (i <= m/2)
            Y(i) = X(i) - 10*rand(1,1) + 5;
        elseif (i > m/2)
            Y(i) = (35 - X(i)) + 10*rand(1,1);
        end
    end
elseif (K == 5)
    way = 1;
    Vx = zeros(1,m);
    Vy = zeros(1,m);
    X = [35*rand(1,m)-30];
    Y = [2*rand(1,m)+0.5];
end

% Setting u - the initial position and velocity vector %

for i = 1:(m+mb) %2m*1 Matrix, mit den Elementen der beiden Geschwindigkeiten in x-und y-Richtung hintereinander (zuerst x, dann y)
    u((2*i)-1) = Vx(i);
    u((2*i)) = Vy(i);
end
for i = 1:(m+mb) %u wird erweritert zu einem 4m*1-Vektor wobei in den 2m-neuen Elementen die X- und Y-Werte abgespreichert werden (zuerst X dann Y)
    u((2*(m+mb))+(2*i)-1) = X(i);
    u((2*(m+mb))+(2*i)) = Y(i);
end
start = u;
%----------------------------------------------------------------------%
% Desired destination - p is initially set as a waypoint (if required) %
%----------------------------------------------------------------------%
% p ist der

if (K == 1)
    p = [[10.5*ones(1,(m+mb)/2),9.5*ones(1,(m+mb)/2)]',[4.5*ones(1,(m+mb))]']; %1*2m 1-er-Matrix, wobei der erste Viertel gleich 10.5, der zweite gleich 9.5 und der Dritte und vierte 4.5
    p1 = [[50*ones(1,(2/0.1)+1)]',[3.5:0.1:5.5]']; %eine zwei mal 21 Matrix (2 Spalten, 21 Zeilen)
    p2 = [[-30*ones(1,(2/0.1)+1)]',[3.5:0.1:5.5]'];
elseif (K == 2)
    p = zeros((m+mb),2);
    %p1 ist das zielarray der linksstartenden, ausserhalb des sichtbaren
    %bereichs ganz rechts, p2 für die rechtsstartenden und p wird erstmal leer
    %definiert und dann wird in der fun5 funktion jedem fussgänger entweder
    %das eine oder andere zugeordnet
    p1 = [[100*ones(1,(6/0.1)+1)]',[7:0.1:13]'];
    p2 = [[zeros(1,(6/0.1)+1)]',[7:0.1:13]'];
elseif (K == 3)
    p = [[5*ones(1,m/2),15*ones(1,m/2)]',[5*ones(1,m)]'];
    p1 = [[20*ones(1,(4/0.1)+1)]',[3:0.1:7]'];
    p2 = [[zeros(1,(4/0.1)+1)]',[3:0.1:7]'];
elseif (K == 4)
    p = zeros(m,2);
    p1 = [[50:0.1:55]',[55:-0.1:50]'];
    p2 = [[-15:0.1:-10]',[50:0.1:55]'];
elseif (K == 5)
    p = [[8.5*ones(1,m)]',[2*ones(1,m)]'];
    p1 = [[7.5:0.1:9.5]',[40*ones(1,(2/0.1)+1)]'];
    p2 = [[7.5:0.1:9.5]',[40*ones(1,(2/0.1)+1)]'];
end
%----------------%
% Boundary Array %
%----------------%

if (K == 1)
    bdry = [[0:0.1:20,0:0.1:20,10*ones(1,(1/0.1)+1),10*ones(1,(1/0.1)+1)...
        ]',[3*ones(1,(20/0.1)+1),6*ones(1,(20/0.1)+1),3:0.1:4,5:0.1:6]',];
    BDRY = 1;
    %Das für uns wichtige Boundary Array erstellt einfach eine Matrix 2x201
    %Matrix und setzt im 2D Raum eine obere und eine Untere Grenze, jeweils
    %auf höhe 1 und auf höhe 6. geplottet wird das ganze dann mit 'rx' kleinen
    %kreuzchen. Die erste Spalte ist die x-Achse, die zweite die Y-Achse
elseif (K == 2)
    bdry = [    [-50:0.1:150   ,           -50:0.1:150]'...
        ,[6*ones(1,(200/0.1)+1)    ,    14*ones(1,(200/0.1)+1)]'];
    BDRY = 1;
elseif (K == 3)
    bdry = [[5:0.1:15,5:0.1:15,5*ones(1,(3/0.1)+1),5*ones(1,(3/0.1)+1)...
        ,15*ones(1,(3/0.1)+1),15*ones(1,(3/0.1)+1)]'...
        ,[3*ones(1,(10/0.1)+1),7*ones(1,(10/0.1)+1),0:0.1:3,7:0.1:10,...
        0:0.1:3,7:0.1:10]'];
    BDRY = 1;
elseif (K == 4)
    BDRY = 0;
elseif (K == 5)
    bdry = [[-30:0.1:10,-30:0.1:7,10*ones(1,(40/0.1)+1),...
        7*ones(1,(37/0.1)+1)]',[zeros(1,(40/0.1)+1),3*ones(1,(37/0.1)+1)...
        ,0:0.1:40,3:0.1:40]'];
    BDRY = 1;
end


%----------------------------------------%
% The solver for the system - here ode45 %
%----------------------------------------%
%options 1 setzt die optionen für's lösen der ode fest, zum bsp mit abstol
%die absolute toleranz und retol die relative
options1 = odeset('AbsTol',1d-3,'RelTol',1d-4);
[t1,u1] = ode45(@fun5m,tspan,start,options1);

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
if (K == 4)
    hold on 
else
    plot(bdry(:,1),bdry(:,2),'rx')
    if (K == 1)||(K==3)
        axis([0,20,0,10]);
    elseif (K == 2)
        axis([25,75,0,20]);
    elseif (K == 5)
        axis([0,10,0,10]);
    end
    hold on
end

% Plotting the pedestrian paths %
%die jeweiligen Pfade der Fussgänger sind abgelegt in einer 2x2 Matrix u,
%für jeden Zeitschritt einen x und y wert. es gibt
for i = ((2*(m*mb)+1):2:(2*(m+mb)+m))
        plot(u1(:,i),u1(:,i+1),'b-') 
            axis([25,75,0,20])   
        hold on
end
   for i = ((2*(m+mb)+m+1):2:(2*(m+mb)+2*m))
        plot(u1(:,i),u1(:,i+1),'r-')
      
            axis([25,75,0,20])
       
        
        hold on
   end
   for i = ((2*(m+mb)+2*m+1):2:(2*(m+mb)+2*m+mb))
       plot(u1(:,i),u1(:,i+1),'g-')
       axis([25,75,0,20])
       hold on
   end
   for i = ((2*(m+mb)+2*m+mb+1):2:(2*(m+mb)+2*m+2*mb))
       plot(u1(:,i),u1(:,i+1),'m-')
       axis([25,75,0,20])
       hold on
   end
   
pause
%-----------------------------------------------------------%
% Making the Movie - plots the pedestrian positions at each %
% timestep and then uses each plot as a movie frame %
%-----------------------------------------------------------%

for j = 1:length(t1)
    % Plotting the boundaries %
    if (K == 4)
        hold off
    else
        hold off
        plot(bdry(:,1),bdry(:,2),'rx')
        if (K == 1)||(K == 3)
            axis([0,20,0,10]);
        elseif (K == 2)
            axis([25,75,0,20]);
        elseif (K == 5)
            axis([0,10,0,10]);
        end
        hold on 6
    end
    % Plotting the pedestrians %
    for i = ((2*(m+mb)+1):2:(2*(m+mb)+m))
        plot(u1(j,i),u1(j,i+1),'bo')
        axis([25,75,0,20])
        hold on
    end
    for i = ((2*(m+mb)+m+1):2:(2*(m+mb)+2*m))
        plot(u1(j,i),u1(j,i+1),'ro')
        axis([25,75,0,20])
        hold on
    end
    for i = ((2*(m+mb)+2*m+1):2:(2*(m+mb)+2*m+mb))
        plot(u1(j,i),u1(j,i+1),'go')
        axis([25,75,0,20])
        hold on
    end
    
    for i = ((2*(m+mb)+2*m+mb+1):2:(2*(m+mb)+2*(m+mb)))
        plot(u1(j,i),u1(j,i+1),'mo')
        axis([25,75,0,20])
        hold on
    end
    F(j) = getframe;
end

clf %clf deletes from the current figure all graphics objects whose handles are not hidden (i.e., their HandleVisibility property is set to on).