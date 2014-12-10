function [t,U,unfalltotal,unfallorttotal] = Iteration_master(T,start)
global m dt k M 
%T, die Simmulationsdauer
%dt, die Iterationsgenauigkeit
%start, beinhaltet die Anfangskoordinaten und Geschwindikgeiten (f�r t=0)
t=[0:dt:T]';    %der dt-Zeitvektor mit allen Iterationszeitschritten
U=zeros(length(t),2*m);   %in U werden die Koordianten von jeder Person zu jedem Zeitpunkt abgespeichert
u=start;    %in u werden immer die aktuellen Koordinaten und Geschwindigkeiten abgespeichert. u=start f�r (t=0)
unfalltotal = 0;
unfallorttotal = zeros(0,2);
uv = zeros(1,m);

i=1;    %Hier wird der Fall t=0 separat betrachtet, um auszuschliessen, dass �berdimensional grosse Kr�fte bei ung�nstiger Anfangsbedingung entstehen

    [f,unfalltotal,unfallorttotal,uv] = fun5_master(t(i),u,unfalltotal, unfallorttotal,uv);     %f, ein 2m*1-Kr�ftevektor. 
    fd = zeros(1,2*m);    %Eine D�mpfung nach fd=-k*v in vertikale Richtung, weil der Mensch von sich aus die Richtung senkrecht zum Ziel zu bremsen beginnt.
    for j=2:2:2*m
       fd(j)= u(j)*-k;   %nur die y-komponente erh�lt eine D�mfpung proportional zur aktuellen Geschwindigkeit in y-Richtung
    end
    f=f+fd;
    dv=f*dt;     %F=dp/dt=(m=1)*dv/dt <=> dv=f*dt, die Geschwindigkeits�nderung ist proportional zur Kraft
    %Hier wird ber�cksichtigt mit der Massenmatrix, dass Velofahrer und
    %Fussg�nger unterschiedliche Massen haben...
    for j=2:2:2*m
        dv(j-1)=dv(j-1)/M(j/2);
        dv(j)=dv(j)/M(j/2);
    end   
    u(1:2*m) = u(1:2*m) + dv;    %die Geschwindikgeits�nderung wird zur vorherigen Geschwindigkeit addiert
    for j=1:1:2*m           %F�r jede Komponente
        if(u(1,j)*dt>2)     %Falls die Orts�nderung f�r t=td gr�sser als 2 ist, soll sie auf eins zur�ckgesetzt werden
            u(1,j)=1/dt;
        end
    end
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt; 
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt;   %Die Orts�nderung do=v*dt wird zum ort hinzuaddiert
    U(i,:) = u(2*m+1:4*m);  %der Ort zum Zeitpunkt t(i) wird in U abgespeichert auf der i-ten Zeile




for i=2:length(t)
    [f,unfalltotal, unfallorttotal, uv] = fun5_master(t(i),u,unfalltotal, unfallorttotal, uv);     %f, ein 2m*1-Kr�ftevektor. 
  %Eine D�mpfung in vertikale Richtung, weil der Mensch von sich aus die Richtung senkrecht zum Ziel zu bremsen beginnt.
    for j=2:2:2*m
       u(j)=u(j)*k;   %nur die y-komponente wird um den Faktor k (zwischen null und eins) ged�mpft
    end
    dv=f*dt;     %F=dp/dt=(m=1)*dv/dt <=> dv=f*dt, die Geschwindigkeits�nderung ist proportional zur Kraft
    %Hier wird ber�cksichtigt mit der Massenmatrix, dass Velofahrer und
    %Fussg�nger unterschiedliche Massen haben...
    for j=2:2:2*m
        dv(j-1)=dv(j-1)/M(j/2);
        dv(j)=dv(j)/M(j/2);
    end
    u(1:2*m) = u(1:2*m) + dv;    %die Geschwindikgeits�nderung wird zur vorherigen Geschwindigkeit addiert
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt;   %Die Orts�nderung do=v*dt wird zum ort hinzuaddiert
    U(i,:) = u(2*m+1:4*m);  %der Ort zum Zeitpunkt t(i) wird in U abgespeichert auf der i-ten Zeile
    unfalltotal = unfalltotal;
    unfallorttotal = unfallorttotal;
    uv = uv;
end
    %die Koordinaten zu jedem Zeitpunkt von jeder Person wird als Funktionswert zur�cktgegeben
