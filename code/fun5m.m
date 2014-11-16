% Driving force, pedestrian interaction and boundary interaction function % %-------------------------------------------------------------------------%
function f0a = fun5m(t,u)
global m mb bdry p p1 p2 D way BDRY x0
global tau taub v0a0 v0a0b vmax vmaxb r A1a A1ab A2a A2ab B1a B1ab B2a B2ab lambda lambdab A1 A1b B1 B1b VS VSb dist
%---------------------------------------------------% % Putting pedestrian positions in a separate vector % %---------------------------------------------------%
%x = zeros(m+mb,2);
for i = (1:(m+mb))
    x(i,1) = u((2*i)-1+(2*m+2*mb)); %Positionen  der Fussgänger und Velofahrer werden in seperaten Vektor getan
    x(i,2) = u((2*i)+(2*m+2*mb));
end %-------------------------------------------------------% % Setting inital position for average speed calculation % %-------------------------------------------------------%
if (t == 0)
    x0 = x;
end

%-------------------------------%
% Desired Velocity from (2.6.6) %
%-------------------------------%
%v0a = zeros(m+mb);
%V = zeros(m+mb,2);
for i = 1:(m+mb)
    if t == 0
        if i <= m 
            v0a(i) = v0a0;
        end
        if i > m
            v0a(i) = v0a0b;
        end
    else
        if i <= m
            V (i,:) = x(i,:) - x0(i,:);
            Vbar = norm(V(i,:)/t);% durchschnittsgeschkeit bis  zeitpkt t 
            na = 1 - (Vbar/v0a0);
            v0a(i) = (1 - na)*v0a0 + na*vmax;
        end
        if i > m
            V(i,:) = x(i,:) - x0(i,:);
            Vbar = norm(V(i,:)/t);
            na = 1 - (Vbar/v0a0b);
            v0a(i) = (1 - na)*v0a0b + na*vmaxb;  
        end
    end
    %-----------------------%
    % Desired destination p %
    %-----------------------%
    %nrmpx = zeros(m+mb);
    if (way == 1)
        % Pedestrians in area A (L-Hand corner for Scramble Crossing) %
        if (i <= m/2 || m < i <= (m+(mb/2)))
            nrmpx(i) = norm(p(i,:) - x(i,:)); %abstand zum ziel
            if (nrmpx(i) < dist)%dist ist der Radius um das Ziel innarhalb welchem das Ziel als erreicht gilt
                D(i) = 1;% ziel erreicht
            end
            if (D(i) == 1)
                p(i,:) = p1(1,:);
                nrmpx(i) = norm(p(i,:) - x(i,:));
                for j = 1:length(p1)
                    if (norm(p1(j,:) - x(i,:)) < nrmpx(i))%es wird jede Stelle in der Zielmatrix durchgegangen um zu schauen welches die Nächste stelle ist, ist sie näher, als die gespeicherte wird sie als neuer zeilpunkt definiert
                        p(i,:) = p1(j,:);
                        nrmpx(i) = norm(p(i,:) - x(i,:));% neue distanz wird berechnet
                    end
                end
            end
            % Pedestrians in area B (R-Hand corner for Scramble Crossing) %
        elseif (m/2 < i <= m || i > (m+(mb/2)))
            nrmpx(i) = norm(p(i,:) - x(i,:));
            if (nrmpx(i) < dist)
                D(i) = 1;
            end
            if (D(i) == 1)
                p(i,:) = p2(1,:);
                nrmpx(i) = norm(p(i,:) - x(i,:));
                for j = 1:length(p2)
                    if (norm(p2(j,:) - x(i,:)) < nrmpx(i))
                        p(i,:) = p2(j,:);
                        nrmpx(i) = norm(p(i,:) - x(i,:));
                    end
                end
            end
        end
    elseif (way == 0) %ziel noch nicht erreicht??? was hat es mit dieser Variabel auf sich
        % Pedestrians in area A (L-Hand corner for Scramble Crossing) %
        if (i <= m/2 || m < i <= (m+(mb/2)))
            p(i,:) = p1(1,:);
            nrmpx(i) = norm(p(i,:) - x(i,:));
            for j = 1:length(p1)
                if (norm(p1(j,:) - x(i,:)) < nrmpx(i))
                    p(i,:) = p1(j,:);
                    nrmpx(i) = norm(p(i,:) - x(i,:));
                end %es wurde wieder der nächste Pkt.in der Zielmatrix eruiert
                
            end
            % Pedestrians in area B (R-Hand corner for Scramble Crossing) %
        elseif ((m/2) < i < m || i > m+(mb/2))
            p(i,:) = p2(1,:);
            nrmpx(i) = norm(p(i,:) - x(i,:));
            for j = 1:length(p2)
                if (norm(p2(j,:) - x(i,:)) < nrmpx(i))
                    p(i,:) = p2(j,:);
                    nrmpx(i) = norm(p(i,:) - x(i,:));
                end
            end
        end
    end
    %---------------------------------------%
    % Desired Direction vector from (2.6.5) %
    %---------------------------------------%
    %e = zeros(m+mb,2);
    if (nrmpx(i) == 0)%Person i ist angekommen
        e(i,:) = [0,0];
    else
        e(i,:) = (p(i,:) - x(i,:))/(nrmpx(i));
    end
    %--------------------------------------%
    % Driving Force component from (2.6.4) %
    %--------------------------------------%
    %f = zeros(2*(m+mb));
    if i <= m
        f((2*i)-1) = ((v0a(i)/tau) * e(i,1)) - (u((2*i)-1)/tau);%v0a ist die gewünschte schnelligkeit, e die gewünschte bewgungsrichtung und u die aktuelle Geschw'keit der person i in X richtung
        f((2*i)) = ((v0a(i)/tau) * e(i,2)) - (u((2*i))/tau);%... der Person i in y Richtung
    end
    if i > m
        f((2*i)-1) = ((v0a(i)/taub) * e(i,1)) - (u((2*i)-1)/taub);%v0a ist die gewünschte schnelligkeit, e die gewünschte bewgungsrichtung und u die aktuelle Geschw'keit der person i in X richtung
        f((2*i)) = ((v0a(i)/taub) * e(i,2)) - (u((2*i))/taub);
    end

    
    %---------------------------------------------------------------------%
    % Pedestrian Interactions from (2.6.8) - Summing all the interactions %
    %---------------------------------------------------------------------%
    %Froce interaction am anfang 0
    fab = [0,0];
    for j = 1:(m+mb)
        if (i ~= j)%abstand zu eigener person nicht berücksichtigen
            dab = norm(x(i,:) - x(j,:));%Abstand von Person i zu Person j
            if (i <= m && dab < VS)% wenn distanz innerhalb des Interaktionsbereich liegt
                rab = r(i) + r(j);%summe der beiden radien der Personen
                nab = ((x(i,:) - x(j,:))/dab);% nab: normalisierter Vektor, der von Person j zu Person i zeigt, dab: ist die Distanz der Massenzentren zweier Personen
                fab = fab + ((A1a*exp((rab - dab)/B1a))*nab)...             %Funktion, die aus dem abstand, radius, richtung, lambda die kraft ausdrückt
                    *(lambda + (1 - lambda)*((1+(-nab*e(i,:)'))/2))...
                    + ((A2a*exp((rab - dab)/B2a))*nab);
            end
            % für die Velofahrer %
            if (i > m && dab < VSb)
                rab = r(i) + r(j);%summe der beiden radien der Personen
                nab = ((x(i,:) - x(j,:))/dab);% nab: normalisierter Vektor, der von Person j zu Person i zeigt, dab: ist die Distanz der Massenzentren zweier Personen
                fab = fab + ((A1ab*exp((rab - dab)/B1ab))*nab)...             %Funktion, die aus dem abstand, radius, richtung, lambda die kraft ausdrückt
                    *(lambdab + (1 - lambdab)*((1+(-nab*e(i,:)'))/2))...
                    + ((A2ab*exp((rab - dab)/B2ab))*nab);
            end
        end
    end
    f((2*i)-1) = f((2*i)-1) + fab(1,1); %f: eigener Antrieb, um gewünschte gesch'keit zu haben, fab Kraft, die all die anderen Personen auf Person i ausüben in X richtung
    f((2*i)) = f((2*i)) + fab(1,2); %in Y Richtung
    %---------------------------------------------------------------------%
    % Boundary Interaction component, this is analogous to the pedestrian %
    % interactions - infact the formula only needs different constants %
    %---------------------------------------------------------------------%
    %-----------------------------------------------------%
    % Summing all the boundary interactions from (2.6.12) %
    %-----------------------------------------------------%
    if BDRY == 1
        if i <= m
            for j = 1:length(bdry)
                if (j == 1)
                    daB = norm(x(i,:) - bdry(j,:));% abstand von person i zum Pkt j,: der/des Wand/Hinderniss
                    BDY = bdry(j,:);
                    rd = r(i) - daB;
                elseif ((norm(x(i,:) - bdry(j,:))) < daB)
                    daB = norm(x(i,:) - bdry(j,:));
                    
                    BDY = bdry(j,:);
                    rd = r(i) - daB;
                end
            end
            naB = (x(i,:) - BDY)/daB;
            faB = ((A1*exp(rd/B1))*naB);
            f((2*i)-1) = f((2*i)-1) + faB(1,1);
            f((2*i)) = f((2*i)) + faB(1,2);
        end
        if i > m
            for j = 1:length(bdry)
                if (j == 1)
                    daB = norm(x(i,:) - bdry(j,:));% abstand von person i zum Pkt j,: der/des Wand/Hinderniss
                    BDY = bdry(j,:);
                    rd = r(i) - daB;
                elseif ((norm(x(i,:) - bdry(j,:))) < daB)
                    daB = norm(x(i,:) - bdry(j,:));
                    
                    BDY = bdry(j,:);
                    rd = r(i) - daB;
                end
            end
            naB = (x(i,:) - BDY)/daB;
            faB = ((A1b*exp(rd/B1b))*naB);
            f((2*i)-1) = f((2*i)-1) + faB(1,1);
            f((2*i)) = f((2*i)) + faB(1,2);
        end
    end
    %----------------------------------%
    % Displacement update from (2.6.1) %
    %----------------------------------%
    X((2*i)-1)= u((2*i)-1);
    X(2*i) = u(2*i);
end
f0a = [f,X]';