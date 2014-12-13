%Dieses Skrip soll das Social_force_giaco mehrere male ausführen und dann
%einen Plot bzw. Histogramm erstelle
 clear all

%die bounderies:
bdry = [    [-50:0.1:150   ,    -50:0.1:150]'...
    ,[6*ones(1,(200/0.1)+1)    ,    14*ones(1,(200/0.1)+1)]'];
%die Velospur
vspur = [[-50:0.1:150]',[9*ones(1,(200/0.1)+1)]']; % raumilche Anordnun

a = 1; % i sollte immer durch 3 teilbar sein
b = 1; 
c = 2;

w = 2; %Anzahl realisationen einer 'Randbedingung?
dauer = 1; %dauer der jeweiligen simulationen
ua = zeros(w+1,0); % matrix mit allen unfalls zahlen aller realisierungen
ue = zeros(w+1,0);
uo = zeros(200,c*2); % die unfallorte matrix mit den realisationen der verschiedenen Randbedingungen
%-------gleicher Anteil von links sowie von rechts--------------% ohne line
%----------------------------------------------------------------------------%
line = 0; % 0 = keine line, 1 = es hat eine line
for i = 1:c
    fl = 4*i; % Fussgänger von links
    fr = 4*i; % Fussgänger von rechts
    vl = 1*i; % velos von links
    vr = 1*i;% velos von rechts
    ufv = zeros(w+1,1); % die Verteilung der Unfälle ppro 'Randbedingung'. 
    anz = fl + fr + vl + vr; % Anzahl agenten
    erreicht = zeros(w+1,1);
    erreicht(1) = anz;
    ufv(1) = anz;
    unfallorte = zeros(0,2);
    for j = 1:w % jede 'Randbedingung' wird w mal realisiert
        [unfaelle, us, Z] = Social_force_master_auswerter(fl,fr,vl,vr,line,dauer); % us ist ein Vektor mit den unfallstellen
        ufv(j+1) = unfaelle;
        unfallorte = [unfallorte; us];
        erreicht(j+1) = sum(Z);
     end
    ua = [ua, ufv]; % ua ist eine w+1 mal n matrix ist
    uo(1:length(unfallorte),(i*2-1):2*i) = unfallorte;
    ue = [ue,erreicht];
    
     
end

%mittelwerte bilden
for i = 1:c
    mittel(i) = mean(ua(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ua(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ua(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ua(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number of danger situations');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+staw(c)+20]);
title('equal from left and rigth, no line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'equal from left and rigth, no line, ped.: bikes = 4:1', 'jpg');

%--- Statistik erreicht---%

for i = 1:c
    mittel(i) = mean(ue(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ue(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ue(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ue(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number that reached the goal');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+10]);
title('equal from left and rigth, no line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'reached equal from left and rigth, no line, ped. bikes = 4 1', 'jpg');


%-----unfallorte-----%
% plotten wir mal alle 
    % Plotting the boundaries %
    %gesamt anzahl unfälle pro simulation bzw 'Randbedingung'
    ges = zeros(2,c);
    for i = 1:c
        ges(1,i) = ua(1,i);
        ges(2,i) = sum(ua(2:w+1,i));
    end
        hold off
        plot(bdry(:,1),bdry(:,2),'rx')
        axis([25,75,0,20]);
        hold on
        for ii = 1:c
            for j=1:ges(2,c); % in ges(2,:) sind jeweils die anzahl unfälle gespeichert, die mit diesen Randbedingungen geschehen
                plot(uo(j,2*ii-1),uo(j,2*ii),'rX')
            end
        end

title('all dangerous situations, equel left and right, all boundery Cond.' ,'FontSize',14);
saveas(gcf, 'all dangerous situations, equel left and right, all start Cond', 'jpg');

complete = 0.25


%------------doppelte Anzahl von links wie von rechts--------% ohne line---%
%--------------------------------------------------------------------------%
ua = zeros(w+1,0); % matrix mit allen unfalls zahlen aller realisierungen
uo = zeros(200,c*2); % die unfallorte matrix mit den realisationen der verschiedenen Randbedingungen

line = 0; % 0 = keine line, 1 = es hat eine line
for i = 1:c
    fl = ceil(4*i*1.25); % Fussgänger von links
    fr = ceil(4*i*0.75); % Fussgänger von rechts
    vl = ceil(1*i*1.25); % velos von links
    vr = ceil(1*i*0.75);% velos von rechts
    ufv = zeros(w+1,1); % die Verteilung der Unfälle ppro 'Randbedingung'. 
    anz = fl + fr + vl + vr; % Anzahl agenten
    ufv(1) = anz;
    erreicht = zeros(w+1,1);
    erreicht(1) = anz;
    unfallorte = zeros(0,2);
    for j = 1:w % jede 'Randbedingung' wird w mal realisiert
        [unfaelle, us, Z] = Social_force_master_auswerter(fl,fr,vl,vr,line,dauer); % us ist ein Vektor mit den unfallstellen
        ufv(j+1) = unfaelle;
        unfallorte = [unfallorte; us];
        erreicht(j+1) = sum(Z);
     end
    ua = [ua, ufv]; % ua ist eine w+1 mal n matrix ist
    uo(1:length(unfallorte),(i*2-1):2*i) = unfallorte;
    ue = [ue,erreicht];
     
end
%mittelwerte bilden
for i = 1:c
    mittel(i) = mean(ua(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ua(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ua(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ua(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number of danger situations');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+staw(c)+20]);
title('double from left, no line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'double from left, no line, ped: bikes = 4:1', 'jpg');

%----erreicht statistik-----%

for i = 1:c
    mittel(i) = mean(ue(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ue(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ue(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ue(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number that reached the goal');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+10]);
title('double from, no line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'reached double from left, no line, ped.: bikes = 4:1', 'jpg');

%-----unfallorte-----%
% plotten wir mal alle 
    % Plotting the boundaries %
    %gesamt anzahl unfälle pro simulation bzw 'Randbedingung'
    ges = zeros(2,c);
    for i = 1:c
        ges(1,i) = ua(1,i);
        ges(2,i) = sum(ua(2:w+1,i));
    end
        hold off
        plot(bdry(:,1),bdry(:,2),'rx')
        axis([25,75,0,20]);
        hold on
        for ii = 1:c
            for j=1:ges(2,c); % in ges(2,:) sind jeweils die anzahl unfälle gespeichert, die mit diesen Randbedingungen geschehen
                plot(uo(j,2*ii-1),uo(j,2*ii),'rX')
            end
        end

title('all dangerous situations, double left, all boundery Cond.' ,'FontSize',14);
saveas(gcf, 'all dangerous situations, double left, all start Cond', 'jpg');

complete = 0.5

%------------gleich viele von links und rechts--------% mit line----%
%-------------------------------------------------------------------%
ua = zeros(w+1,0); % matrix mit allen unfalls zahlen aller realisierungen
uo = zeros(200,c*2); % die unfallorte matrix mit den realisationen der verschiedenen Randbedingungen

line = 1; % 0 = keine line, 1 = es hat eine line
for i = 1:c
    fl = 4*i; % Fussgänger von links
    fr = 4*i; % Fussgänger von rechts
    vl = 1*i; % velos von links
    vr = 1*i;% velos von rechts
    ufv = zeros(w+1,1); % die Verteilung der Unfälle ppro 'Randbedingung'. 
    anz = fl + fr + vl + vr; % Anzahl agenten
    ufv(1) = anz;
    erreicht = zeros(w+1,1);
    erreicht(1) = anz;
    unfallorte = zeros(0,2);
    for j = 1:w % jede 'Randbedingung' wird w mal realisiert
        [unfaelle, us, Z] = Social_force_master_auswerter(fl,fr,vl,vr,line,dauer); % us ist ein Vektor mit den unfallstellen
        ufv(j+1) = unfaelle;
        unfallorte = [unfallorte; us];
        erreicht(j+1) = sum(Z);
     end
    ua = [ua, ufv]; % ua ist eine W+1 mal n matrix ist
    uo(1:length(unfallorte),(i*2-1):2*i) = unfallorte;
    ue = [ue,erreicht];     
end
%mittelwerte bilden
for i = 1:c
    mittel(i) = mean(ua(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ua(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ua(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ua(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number of danger situations');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+staw(c)+20]);
title('equal from left and right, line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'equel from left and right, line, ped: bikes = 4:1', 'jpg');

%----erreicht Statistik-----%

for i = 1:c
    mittel(i) = mean(ue(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ue(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ue(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ue(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number that reached the goal');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+10]);
title('equal from left and rigth, with line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'reached equal from left and rigth, with line, ped.: bikes = 4:1', 'jpg');

%-----unfallorte-----%
% plotten wir mal alle 
    % Plotting the boundaries %
    %gesamt anzahl unfälle pro simulation bzw 'Randbedingung'
    ges = zeros(2,c);
    for i = 1:c
        ges(1,i) = ua(1,i);
        ges(2,i) = sum(ua(2:w+1,i));
    end
       
    % Plotting the boundaries %
        hold off
        plot(bdry(:,1),bdry(:,2),'rx')
        hold on 
        plot(vspur(:,1),vspur(:,2),'r-')
        axis([25,75,0,20]);
        hold on
        for ii = 1:c
            for j=1:ges(2,c); % in ges(2,:) sind jeweils die anzahl unfälle gespeichert, die mit diesen Randbedingungen geschehen
                plot(uo(j,2*ii-1),uo(j,2*ii),'rX')
            end
        end

title('all dangerous situations, equal left right, all boundery Cond.' ,'FontSize',14);
saveas(gcf, 'all dangerous situations, equal left right, all start Cond with line', 'jpg');

complete = 0.75

%------------doppelt so viele von links wie von rechts--------% mit line-%
%------------------------------------------------------------------------%

ua = zeros(w+1,0); % matrix mit allen unfalls zahlen aller realisierungen
uo = zeros(200,c*2); % die unfallorte matrix mit den realisationen der verschiedenen Randbedingungen

line = 1; % 0 = keine line, 1 = es hat eine line
for i = 1:c
    fl = ceil(4*i*1.25); % Fussgänger von links
    fr = ceil(4*i*0.75); % Fussgänger von rechts
    vl = ceil(1*i*1.25); % velos von links
    vr = ceil(1*i*0.75);% velos von rechts
    ufv = zeros(w+1,1); % die Verteilung der Unfälle ppro 'Randbedingung'. 
    anz = fl + fr + vl + vr; % Anzahl agenten
    ufv(1) = anz;
    erreicht = zeros(w+1,1);
    erreicht(1) = anz;
    unfallorte = zeros(0,2);
    for j = 1:w % jede 'Randbedingung' wird w mal realisiert
        [unfaelle, us, Z] = Social_force_master_auswerter(fl,fr,vl,vr,line,dauer); % us ist ein Vektor mit den unfallstellen
        ufv(j+1) = unfaelle;
        unfallorte = [unfallorte; us];
        erreicht(j+1) = sum(Z);
     end
    ua = [ua, ufv]; % ua ist eine w+1 mal n matrix ist
    uo(1:length(unfallorte),(i*2-1):2*i) = unfallorte;
    ue = [ue,erreicht];
    
     
end
%mittelwerte bilden
for i = 1:c
    mittel(i) = mean(ua(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ua(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ua(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ua(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number of danger situations');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+10]);
title('double from left, line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'double from left, line, ped.: bikes = 4:1', 'jpg');

%-----erreicht Statistik-----%
for i = 1:c
    mittel(i) = mean(ue(2:w+1,i));
end

for i= 1:c
    staw(i) = std(ue(2:w+1,i));
end
    hold off
for k = 1:c
    plot(ue(1,k),mittel(k),'b.','LineWidth',10, 'MarkerSize', 16);
    hold on
    errorbar(ue(1,k),mittel(k),staw(k),'k','LineWidth',2);
    hold on
end
set(gca,'FontSize',16);
xlabel('number of individuals');
ylabel('number that reached the goal');
xlim([0 ua(1,c)+10]);
ylim([0 ua(w+1,c)+staw(c)+20]);
title('double from left, with line, ped.: bikes = 4:1','FontSize',14);
saveas(gcf, 'reached double from left, with line, ped.: bikes = 4:1', 'jpg');

%-----unfallorte-----%
% plotten wir mal alle 
    % Plotting the boundaries %
    %gesamt anzahl unfälle pro simulation bzw 'Randbedingung'
    ges = zeros(2,c);
    for i = 1:c
        ges(1,i) = ua(1,i);
        ges(2,i) = sum(ua(2:w+1,i));
    end
       
    % Plotting the boundaries %
        hold off
        plot(bdry(:,1),bdry(:,2),'rx')
        hold on 
        plot(vspur(:,1),vspur(:,2),'r-')
        axis([25,75,0,20]);
        hold on
        for ii = 1:c
            for j=1:ges(2,c); % in ges(2,:) sind jeweils die anzahl unfälle gespeichert, die mit diesen Randbedingungen geschehen
                plot(uo(j,2*ii-1),uo(j,2*ii),'rX')
            end
        end

title('all dangerous situations, double left, all boundery Cond.' ,'FontSize',14);
saveas(gcf, 'all dangerous situations, double left, all start Cond, with line', 'jpg');





 
