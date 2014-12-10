% Driving force, pedestrian interaction and boundary interaction function % %-------------------------------------------------------------------------%
function [f0a,unfalltotal,unfallorttotal,uv] = fun5_master(t,u,unfalltotal,unfallorttotal, uv)
global m bdry p p1 p2   x0 L ml mr Av Bv vspur VF1 VF2 VV1 VV2 line
global tau v01 v02 vmax1 vmax2 r A1a A2a A3a A4a B1a B2a B3a B4a lambda A1 B1 VS
%---------------------------------------------------% % Putting pedestrian positions in a separate vector % %---------------------------------------------------%
x = zeros(m,2); %damit die grösse von x nicht bei jedem loop zweimal ändert (Speed)
X = zeros(1,2*m); %"", wird ganz unten verwendet (dritt letzte Zeile)   (Speed)

%die Positionen werden in die m*2 Matrix x abgespeichert, wobei die i-te
%Zeile der i-ten Person entspricht
for i = 1:m
    x(i,1) = u((2*i)-1+(2*m));      %X-Koordinaten      Koordinaten stecken in den zweiten 2m von u
    x(i,2) = u((2*i)+(2*m));        %Y-Koordinaten
end %-------------------------------------------------------% % Setting inital position for average speed calculation % %-------------------------------------------------------%
for i= 1:m;
    uv= uv;
end

%da initial t==0, so wird x0 als Startkoordinatenmatrix m*2 in jedem Fall
%als eine globale Variabel definiert; bzw. die if schlaufe wird in jedem
%Fall durchlaufen
if (t == 0)
    x0 = x;
end

%-------------------------------%
% Desired Velocity from (2.6.6) %
%-------------------------------%
%wird für jede Person (m mal) durchlaufen
v0a=v01*ones(1,m); %die aktuelle Geschwindigkeit der i-ten Person. Initialisiert, damit nicht bei jedem Loop geändert werden muss (Speed)
V=zeros(m,2); %Der Orts-Differenzenvektor zwischen Anfangskoord. und momentanen Koord. Initialisierung für Speed
e=zeros(m,2); %Der Einheitsvektor zum am nächsten liegenden Zielpunkt, initialisiert für Speed
f=zeros(1,2*m);%die Kraft, proportional zu Differenz der gewünschten Geschwindigkeit und der Aktuellen Geschwindikgeit. Proportionalitätskonst. ist 1/tau. (Speed)
nrmpx=zeros(1,m);%der momentane Abstand vom i-ten Fussgänger zum momentanen Zielpunkt. (Speed)
for i = 1:m
    
    if t == 0
        for j=ml+1:L
            v0a(j) = v02; %v02 =  Initial desired velocity der Velofahrer$
        end
        
        for k=L+mr+1:m
            v0a(k)=v02;
        end
    else
        V (i,:) = x(i,:) - x0(i,:); %V, eine m*2 Matrix mit dem Orts-Differenzenvektor zwischen Anfangskoordinanten und momentanen Koordinaten
        Vbar = norm(V(i,:)/t);      %norm(V(i,:)/t) nimmt die 2-er-norm (Pythagoras) von der i-ten Zeile;
        %also die bisher zurückgelegte Distanz der i-ten Person.
        %Diese wird durch t geteilt, um die
        %Durchschnittsgeschwindigkeit zu
        %erhalten
        if (i>ml&&i<=L) || (i>L+mr&&i<=m) %filtert die Velos heraus
            na = 1-(Vbar/v02);
            v0a(i) = (1 - na)*v02 + na*vmax2;
        else
            na = 1 - (Vbar/v01);       %Vbar/v01 ist das Verhältnis der bisherigen Durchschnittsgeschw. zur gewünschten Geschwindigkeit. (Ist i.a kleiner als eins)
            v0a(i) = (1 - na)*v01 + na*vmax1;                        %na, der Zeitliche Verzug(i.a. 0<na>1)
        end                         %Je nach dem wie gross der Verzug na, wird  v0a näher zu vmax oder zu v0a0 gerückt --> i.a. v0a0 < v0a > vmax
    end
    
    %-----------------------%
    % Desired destination p %
    %-----------------------%
    
    %Hier wird der zum i-ten von LINKS kommenden Fussgänger sich am nächsten befindenden
    %Zielpunkt (einer der 61) ermittelt, sowie der Betrag dieses Abstandes
    %in nrmpx(i) gespeichert.
    
    if i<=L   %Die erste Hälfte der Personen
        
        p(i,:) = p1(1,:);   %Jedem von links kommenden Fussgänger wird der erste Zielpunkt zugeordnet
        nrmpx(i) = norm(p(i,:) - x(i,:));   %hier wird der aktuelle Abstand von diesem Ziel in nrmpx(i) der i-ten Person berechnet
        for j = 2:length(p1)                %For-loop mit den 61 möglichen anzustrebenden Punkte auf der Ziel-Y-Achse (legnth(p1)=61). Der erste Vergleich muss nicht gemacht werden (trivial)
            if (norm(p1(j,:) - x(i,:)) < nrmpx(i))  %Falls der Abstand zu einem der 61 Zielpunkten zur momentan betrachteten Person i
                %kleiner ist als der Abstand der Person i zum momentanen
                %Ziel
                
                p(i,:) = p1(j,:);       %der somit nähere Zielpunkt wird zum neuen Ziel
                nrmpx(i) = norm(p(i,:) - x(i,:));   %und der neue Abstand wird zum bisher kürzesten Abstand
            end
            
        end
        
        
        
        %Hier das selbe für die von rechts kommenden Fussgänger
    else
        
        p(i,:) = p2(1,:);
        nrmpx(i) = norm(p(i,:) - x(i,:));
        for j = 1:length(p2)
            if (norm(p2(j,:) - x(i,:)) < nrmpx(i))
                p(i,:) = p2(j,:);
                nrmpx(i) = norm(p(i,:) - x(i,:));
            end
        end
    end
    
    %---------------------------------------%
    % Desired Direction vector from (2.6.5) %
    %---------------------------------------%
    if (nrmpx(i) == 0)%Person i ist angekommen
        e(i,:) = [0,0];
    else
        e(i,:) = (p(i,:) - x(i,:))/(nrmpx(i));  %p(i,:)-x(i,:), der momentane Zielpunkt der Person i minus der momentane ort der Person i, geteilt den Betrag dieses Differenzevektors
        %ergibt den Einheitsrichtungsvektor zum Zielpunkt e(i,:)
    end
    %--------------------------------------%
    % Driving Force component from (2.6.4) %
    %--------------------------------------%
    f(2*i-1) = ((v0a(i)/tau) * e(i,1)) - (u(2*i-1)/tau);%v0a ist die gewünschte schnelligkeit, e die gewünschte bewgungsrichtung und u die aktuelle Geschw'keit der person i in X richtung
    f(2*i) = ((v0a(i)/tau) * e(i,2)) - (u(2*i)/tau);%... der Person i in y Richtung
    %je grösser tau, desto kleiner die Kraft
    %---------------------------------------------------------------------%
    % Pedestrian Interactions from (2.6.8) - Summing all the interactions %
    %---------------------------------------------------------------------%
    fab = [0,0]; %Die Kraft, welche von den anderen Fussgänger/Velofahrer auf den i-ten Passant ausgeübt wird (enthält eine X- und Y-Komponente)
    if((i<=ml) || (i>L && i<=L+mr))     %Falls i ein Fussgänger ist...
        for j = 1:m
            if (i ~= j)
                dab = norm(x(i,:) - x(j,:));%Abstand von Person i zu Person j (der beiden Massenzentren)
                if (dab < VS)% wenn distanz innerhalb des Interaktionsbereich liegt
                    rab = r(i) + r(j);%summe der beiden radien der Personen
                    nab = ((x(i,:) - x(j,:))/dab);% nab: normalisierter Vektor, der von Person j zu Person i zeigt. (dab ist die Distanz der Massenzentren der beiden Personen)
                    %                     vab = [u(j),u(j+1)] *nab' - [u(i),u(i+1)] *nab'; %geschwindigkeiten auf die direkte verbindung von i,j projeziert
                    %                     if vab < 0
                    %                         vab = 0;
                    %                     end
                    fab = fab + ((A1a*exp((rab - dab)/B1a))*nab)... %Aufsummierung der Kräfte aller anderen Personen (A1a, Proportionalitätskonstante; (rab-dab) wenn dab klein ist wird dieser ausdruck exponentiell grösser!
                        *(lambda + (1 - lambda)*((1+(-nab*e(i,:)'))/2))... %lambda sagt etwas über die Geometrie aus, des wirkungsbereich (vorne grösser als hinten)
                        + ((A2a*exp((rab - dab)/B2a))*nab); %eine weitere Kraft, welche die physikalische komponente simulieren soll. sie ist unabhängig von lambda, also dem Sichtbereich
                    %Parameter: Ai wachsend verstärken die Kraft, Bi wachsend
                    %schwächt die Kraft
                end
            end
        end
    else    %Falls i ein Velofahrer ist, soll ein eigenes Velofahrerkräftegesetz gelten
        for j=1:m
            %             h=A3a;
            %             if((i<=L && j<=L)||(i>L+mr && j>L+mr))   %Falls Velofahrer in gleiche richtung, keine Abstossende "Sichtkraft"
            %                 h=0;
            %             end
            
            ei=[u(2*i-1),u(2*i)]/norm(u(2*i-1),u(2*i)); %Der Geschwindikeitseinheitsvektorvektor des i-ten Velofahrers
            eij=[x(j,:)-x(i,:)]/norm(x(j,:)-x(i,:));    %Der Einheitsvektor von i zu j
            ex=[1,0];    %der Einheitsvektor in x-Richtung
            %Ellpisengleichung im Parameter phi M(x0,y0), alpha, Verdrehung
            %x=X0+a*cosphi*cosalpha - b*sinphi*sinalpha
            %y=Y0+a*cosphi*sinalpha + b*sinphi*cosphi
            cosphi=eij*ei';
            cosalpha=ei*ex';
            sinphi=sqrt(1-cosphi*cosphi);
            sinalpha=sqrt(1-cosalpha*cosalpha);
            X0=x(i,1);
            Y0=x(i,2);
            b= 1;    %Die kurze halbachse
            %a=b+(15-b)*norm([u(2*i-1),u(2*i)])/vmax2; %Die lange Halbachse muss eine Fkt. der aktuellen Geschwindigkeitn sein. Sie soll
            a=6;
            %proportional mit der Geschwindikgeit zunehmen (weiter voraus
            %schauen bei grösserer Geschwindigkeit)
            
            %Die Koordinaten des Ellipsenrandpunktes
            x1=X0+a*cosphi*cosalpha - b*sinphi*sinalpha;
            y1=Y0+a*cosphi*sinalpha + b*sinphi*cosphi;
            
            P=[x1-X0,y1-Y0];    %Der Vektor vom i-ten Velofahrer zum Ellipsenrand
            L1=norm(P); %Der Abastand vom Ellipsenmittelpunkt bis zum Ellypsenrand
            L2=norm(x(j,:)-x(i,:));     %Der Abstand vom Ellipsenmittelpunkt bis zur j-ten Person
            if((i~=j)&&(cosphi>0)&&(L2<L1));    %cosphi>0 um nur die vordere hälfte der Ellypse zu berücksichtigen
                rab = r(i) + r(j);%summe der beiden radien der Personen
                vab = [u(j),u(j+1)] *(-eij)' - [u(i),u(i+1)] *(-eij)'; %geschwindigkeiten auf die direkte verbindung von i,j projeziert
                vab=vab;
                if vab < 0
                    vab = 0;
                end
                
                fab = fab + (vab*vab)*A3a*exp((rab - L2)/B3a)*-eij... Aufsummierung der Kräfte aller anderen Personen (A1a, Proportionalitätskonstante; (rab-dab) wenn dab klein ist wird dieser ausdruck exponentiell grösser!
                    +((A4a*exp((rab - L2)/B4a))*-eij); %eine weitere Kraft, welche die physikalische komponente simulieren soll. sie ist unabhängig von lambda, also dem Sichtbereich
                %Parameter: Ai wachsend verstärken die Kraft, Bi wachsend
                %schwächt die Kraft
                
            end
        end
    end
    
    
    f((2*i)-1) = f((2*i)-1) + fab(1,1); % in X Richtung: f: eigener Antrieb, um gewünschte geschw'keit zu haben, fab Kraft, die all die anderen Personen auf Person i ausüben in X richtung
    f((2*i)) = f((2*i)) + fab(1,2); %in Y Richtung: "
    
    unfallperson = 0;
    unfallortp = zeros(0,2);
    if uv(i) == 0 % falls im zeitstepp t-1 ein unfall war wurde uv(i) auf 1 gesetzt und somit ist es nicht möglich im zeitstep t ein Unfallzubauen falls in t-1 eienr gebaut wurde
        for j = 1:m;
            dij =norm(x(j,:) - x(i,:)); % Abstand ij
            
            if j ~= i && dij < 0.5;
                
                unfallperson = unfallperson +1; % anzzal unfälle dieser Person
                xu = x(i,:) + (x(j,:) - x(i,:))/2; % Unfallort in der Mitte von i und j
                unfallortp(unfallperson,:) = xu; % unfallorte der Person i
                uv(i) = 1;
            end
            
        end
    
    elseif uv(i) ==1  % falls vorher (t-1) ein unfall war im nächsten zeitschritt soll es möglich sein ein Unfall zu haben falls in diesem Zeitschritt kein unfall ist
        for j = 1:m;
            dij =norm(x(j,:) - x(i,:)); %
        
        
        if j ~= i && dij < 0.5; % wenn ein unfall ist im nächsten zeitschritt wieder nicht möglich einen Unfall zu bauen
            uv(i) = 1;
            
        elseif j ~= i 
            
            uv(i) = 0; %falls jetzt kein Unfall soll es im nächsten Zeitschritt wieder möglich sein einen Unfall zu bauen
            
        end
            
        end
        
    end
    
    
    
    
    
    
    
    %---------------------------------------------------------------------%
    % Boundary Interaction component, this is analogous to the pedestrian %
    % interactions - infact the formula only needs different constants %
    %---------------------------------------------------------------------%
    %-----------------------------------------------------%
    % Summing all the boundary interactions from (2.6.12) %
    %-----------------------------------------------------%
    %Jeder Randpunkt wird abgetastet und geschauht, welcher von inen am
    %nächsten zum i-ten Fussgänger liegt
    daB = norm(x(i,:) - bdry(1,:)); %abstand vom ort der Person zum j-ten Rand
    BDY = bdry(1,:);    %BDY, speichert der momentan am nächsten liegende Randpunkt
    rd = r(i) - daB;    %rd, speichert den (negativen) Abstand vom momentan am nächste liegenden Rand bis zum eigenradius r(i)
    for j = 1:length(bdry) %length bdry=201 boudary-punkte
        if ((norm(x(i,:) - bdry(j,:))) < daB) %wenn der j-te Randpunkt näher ligt als der bisherig als nächsten betrachteten
            daB = norm(x(i,:) - bdry(j,:));  %dann wird daB diesen Wert des kürzeren Abstand annehmen
            BDY = bdry(j,:);    %BDY übernimmt den momentan am nächsten liegenden Randpunkt
            rd = r(i) - daB;    %rd übernimmt den Abstand des näherliegenden Randpunktes zum Eigenradius r(i)
        end
    end
    %nun enthält BDY den am nächsten liegenden Randpunkt und rd den
    %Abstand zu diesem
    
    %hier wird die Randkraft berechnet
    naB = (x(i,:) - BDY)/daB;   %der Einheitsvektor vom nächsten Randpunkt zum Massenzentrum hin
    
    faB = ((A1*exp(rd/B1))*naB);    %die Abstossende Kraft (exponentiell zunehmend) zeigt in diese Richtung des Einheitsvektors und nimmt exponentiell mit wachsendem rd=r(i)-daB ,also negativen Abstand zum Rand, zu.
    %die Kräfte werden zu den anderen Kräfte addiert
    f((2*i)-1) = f((2*i)-1) + faB(1,1);
    f((2*i)) = f((2*i)) + faB(1,2);
    
    %---------------------%
    % Velospur Ineraction %
    %---------------------%--%
    % Fussgänger interaction %
    %------------------------%
    if line == 1
        daV = norm(x(i,:)-vspur(1,:));  % Abstand der Person zum j ten VS- Linien Pkt
        VSR = vspur(1,:); % speichert den am nächsten liegenden Velospur- Linien Pkt.
        vsd = r(i)- daV; % der am nächsten liegende VS linien Pkt. wird abgespeichert
        for j = 1:length(vspur);
            if ((norm(x(i,:) - vspur(j,:))) < daV) %wenn der j-te Randpunkt näher ligt als der bisherig als nächsten betrachteten
                daV = norm(x(i,:) - vspur(j,:));  %dann wird daB diesen Wert des kürzeren Abstand annehmen
                VSR = vspur(j,:);    %BDY übernimmt den momentan am nächsten liegenden Randpunkt
                vsd = r(i) - daV;    %rd übernimmt den Abstand des näherliegenden Randpunktes zum Eigenradius r(i)
            end
        end
        naV = (x(i,:) - VSR)/daV; % Einheits vektro vom nächsten VS Linien Pkt. zu Massenzentrum
        faVS = [0,0];
        if (i<=ml) || (i>L&&i<=L+mr) && x(i,2)>9; %falls ein Fussgänger und ausserhalb der Velospur
            
            faVS = VF1*exp(vsd/VF2)*naV;
        elseif (i<=ml) || (i>L&&i<=L+mr) && x(i,2) <= 9; % falls ein fussgänger und innerhalb der Velospur
            faVS = VF1*exp((r(i)/VF2))*-naV;
        end
        %Velointeractions
        
        if (i>ml && i<L) || (i>L+mr && i<=m) && x(i,2) <= 9; % falls ein Velofahrer und in der Velospur
            faVS = (VV2*(exp(vsd/VV1)))*naV;
        elseif (i>ml && i<L) || (i>L+mr && i<=m) && x(i,2) > 9   % falls ein Velofahrer und nicht in der Velospur
            faVS = (VV2*(exp(r(i)/VV1)))*-naV;
        end
        
        
        %kräfte werden zu den anderen hinzugezählt
        f((2*i)-1) = f((2*i)-1) + faVS(1,1);
        f((2*i)) = f((2*i)) + faVS(1,2);
        
    end
    
    
    
    %---------------------------------------------------------------------%
    % Anziehungskraft zwischen Velofahrern. Drang sich hinter den naechsten, sich weiter vorne befindenden Velofahrer gleicher Zielrichtung zu begeben %
    %---------------------------------------------------------------------%
    if i>ml+1 && i<=L    %nur die Velofahrer der linken Seite
        fv = [0,0]; %Die Anziehungskraft, welche auf den i-ten Velofahrer wirkt
        Dij=100;   %Speichert den kleinsten Abstand (100 kann nicht in diesem Fall nicht überschritten werden)
        k=0;
        for j=ml+1:L
            
            if (i ~= j && (x(j,1)-x(i,1))>0)  %Der j-te Velofahrer muss vor dem i-ten sein, damit der i-te sich hinter ihn gesellen kann
                dij = norm(x(i,:) - x(j,:)); %abstand vom i-ten Velofahrer zum j-ten Velofahrer
                if(dij<Dij)
                    Dij=dij; %Dij \[UDoubleDot]bernimmt den kleineren Abstand zum j-ten Velofahrer
                    k=j; %speichert den am naechsten fahrenden (j-ten) Velofahrer
                end
            end
        end
        if(k~=0)  %wenn ein j-ter Velofahrer vor dem i-ten Velofraher fährt
            if(norm(u(2*k-1),u(2*k))>0)    %Wenn die geschwindigkeit des vorderen nicht null ist
                ek=[u(2*k-1),u(2*k)]/norm(u(2*k-1),u(2*k));  %der Geschw.richtungsvektor des k-ten Velofahrer
                
                ev=[-ek(1,2),ek(1,1)];  %der Vektor senkrecht auf ek
                eik=[x(k,:)-x(i,:)]/norm(x(k,:)-x(i,:));  %der Richtungsvektor vom i-ten zum k-ten Velofahrer
                
                cosphi=ev*eik';
                if cosphi<0
                    ev=-ev;  %das richtige Vorzeichen von ev wird eruiert
                end
                if (dij>3*r(k))     %die senkrechtkraft soll nur wirken, wenn der abstand zum vorderen grösser als zweimal der Eigenradius ist!
                    fv=Av*ev*exp(-Dij/Bv);  %die Senkrechtkraft zeigt in Richtung von ev und ist reziprok zum Abstand vom nächsten Velofahrer, also stärker je näher dieser ist
                else
                    fv=Av*-ek;  %ansonsten soll der hintere Velofahrer etwas abbremsen...
                end
            end
        end
        f((2*i)-1) = f((2*i)-1) + fv(1,1); % in X Richtung: Die Anzihung wird zu den anderern Kr\[ADoubleDot]ften addiert
        f((2*i)) = f((2*i)) + fv(1,2); %in Y Richtung: "
        
    elseif  i>L+ml     % nur die Velofahrer der rechten Seite
        fv = [0,0]; % Die Anziehungskraft, welche auf den i-ten Velofahrer wirkt
        Dij=100;   %Speichert den kleinsten Abstand (100 kann nicht in diesem Fall nicht berschritten werden)
        k=0;
        for j=L+ml+1:m
            if (i ~= j && (x(j,1)-x(i,1))<0)  % Der j-te Velofahrer muss vor dem i-ten sein, damit der i-te sich hinter ihn gesellen kann
                dij = norm(x(i,:) - x(j,:)); % abstand vom i-ten Velofahrer zum j-ten Velofahrer
                if(dij<Dij)
                    Dij=dij; % Dij \[UDoubleDot]bernimmt den kleineren Abstand zum j-ten Velofahrer
                    k=j; % speichert den am n\[ADoubleDot]chsten fahrenden (j-ten) Velofahrer
                end
            end
        end
        if(k~=0)  %wenn ein j-ter Velofahrer vor dem i-ten Velofraher fährt
            if(norm(u(2*k-1),u(2*k))>0)    %Wenn die geschwindigkeit des vorderen nicht null ist
                ek=[u(2*k-1),u(2*k)]/norm(u(2*k-1),u(2*k));  %der Geschw.richtungsvektor des k-ten Velofahrer
                
                ev=[-ek(1,2),ek(1,1)];  %der Vektor senkrecht auf ek
                eik=[x(k,:)-x(i,:)]/norm(x(k,:)-x(i,:));  %der Richtungsvektor vom i-ten zum k-ten Velofahrer
                ev=ev;
                eik=eik;
                cosphi=ev*eik';
                if cosphi<0
                    ev=-ev;  %das richtige Vorzeichen von ev wird eruiert
                end
                if (dij>3*r(k))     %die senkrechtkraft soll nur wirken, wenn der abstand zum vorderen grösser als zweimal der Eigenradius ist!
                    fv=Av*ev*exp(-Dij/Bv);  %die Senkrechtkraft zeigt in Richtung von ev und ist reziprok zum Abstand vom nächsten Velofahrer, also stärker je näher dieser ist
                else
                    fv=Av*-ek;  %ansonsten soll der hintere Velofahrer etwas abbremsen...
                end
            end
        end
        %Die Velofahrer haben eine grössere Masse wie die Fussgänger
        f((2*i)-1) = f((2*i)-1) + fv(1,1); % in X Richtung: Die Anziehung wird zu den anderern Kr\[ADoubleDot]ften addiert
        f((2*i)) = f((2*i)) + fv(1,2); % in Y Richtung: "
    end
    unfalltotal = unfalltotal + unfallperson; % unfälle aller Personen werden zusammengezählt
    unfallorttotal = [unfallorttotal; unfallortp];% unfallorte aller personen werden in eine Matrix gemacht
end


unfalltotal = unfalltotal;
unfallorttotal =unfallorttotal;
uv = uv;

f0a = f;   %gibt eine einspaltige und 2*m zeilige Matrix zurück mit den X-Y-kräften