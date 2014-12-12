function [t,U,unfalltotal,unfallorttotal] = Iteration_master(T,start)
global m dt k M 
%T, die Simmulationsdauer
%dt, die Iterationsgenauigkeit
%start, beinhaltet die Anfangskoordinaten und Geschwindikgeiten (für t=0)
t=[0:dt:T]';    %der dt-Zeitvektor mit allen Iterationszeitschritten
U=zeros(length(t),2*m);   %in U werden die Koordianten von jeder Person zu jedem Zeitpunkt abgespeichert
u=start;    %in u werden immer die aktuellen Koordinaten und Geschwindigkeiten abgespeichert. u=start für (t=0)
unfalltotal = 0;
unfallorttotal = zeros(0,2);
uv = zeros(1,m);

i=1;    %Hier wird der Fall t=0 separat betrachtet, um auszuschliessen, dass überdimensional grosse Kräfte bei ungünstiger Anfangsbedingung entstehen

    [f,unfalltotal,unfallorttotal,uv] = fun5_master(t(i),u,unfalltotal, unfallorttotal,uv);     %f, ein 2m*1-Kräftevektor. 
    fd = zeros(1,2*m);    %Eine Dämpfung nach fd=-k*v in vertikale Richtung, weil der Mensch von sich aus die Richtung senkrecht zum Ziel zu bremsen beginnt.
    for j=2:2:2*m
       fd(j)= u(j)*-k;   %nur die y-komponente erhält eine Dämfpung proportional zur aktuellen Geschwindigkeit in y-Richtung
    end
    f=f+fd;
    dv=f*dt;     %F=dp/dt=(m=1)*dv/dt <=> dv=f*dt, die Geschwindigkeitsänderung ist proportional zur Kraft
    %Hier wird berücksichtigt mit der Massenmatrix, dass Velofahrer und
    %Fussgänger unterschiedliche Massen haben...
    for j=2:2:2*m
        dv(j-1)=dv(j-1)/M(j/2);
        dv(j)=dv(j)/M(j/2);
    end   
    u(1:2*m) = u(1:2*m) + dv;    %die Geschwindikgeitsänderung wird zur vorherigen Geschwindigkeit addiert
    for j=1:1:2*m           %Für jede Komponente
        if(u(1,j)*dt>2)     %Falls die Ortsänderung für t=td grösser als 2 ist, soll sie auf eins zurückgesetzt werden
            u(1,j)=1/dt;
        end
    end
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt; 
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt;   %Die Ortsänderung do=v*dt wird zum ort hinzuaddiert
    U(i,:) = u(2*m+1:4*m);  %der Ort zum Zeitpunkt t(i) wird in U abgespeichert auf der i-ten Zeile




for i=2:length(t)
    [f,unfalltotal, unfallorttotal, uv] = fun5_master(t(i),u,unfalltotal, unfallorttotal, uv);     %f, ein 2m*1-Kräftevektor. 
  %Eine Dämpfung in vertikale Richtung, weil der Mensch von sich aus die Richtung senkrecht zum Ziel zu bremsen beginnt.
    for j=2:2:2*m
       u(j)=u(j)*k;   %nur die y-komponente wird um den Faktor k (zwischen null und eins) gedämpft
    end
    dv=f*dt;     %F=dp/dt=(m=1)*dv/dt <=> dv=f*dt, die Geschwindigkeitsänderung ist proportional zur Kraft
    %Hier wird berücksichtigt mit der Massenmatrix, dass Velofahrer und
    %Fussgänger unterschiedliche Massen haben...
    for j=2:2:2*m
        dv(j-1)=dv(j-1)/M(j/2);
        dv(j)=dv(j)/M(j/2);
    end
    u(1:2*m) = u(1:2*m) + dv;    %die Geschwindikgeitsänderung wird zur vorherigen Geschwindigkeit addiert
    u(2*m+1:4*m)=u(2*m+1:4*m)+u(1:2*m)*dt;   %Die Ortsänderung do=v*dt wird zum ort hinzuaddiert
    U(i,:) = u(2*m+1:4*m);  %der Ort zum Zeitpunkt t(i) wird in U abgespeichert auf der i-ten Zeile
    unfalltotal = unfalltotal;
    unfallorttotal = unfallorttotal;
    uv = uv;
end
    %die Koordinaten zu jedem Zeitpunkt von jeder Person wird als Funktionswert zurücktgegeben
