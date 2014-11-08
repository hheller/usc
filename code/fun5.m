% Driving force, pedestrian interaction and boundary interaction function % %-------------------------------------------------------------------------%
function f0a = fun5(t,u)
global m bdry p p1 p2 D way BDRY x0
global tau v0a0 vmax r A1a A2a B1a B2a lambda A1 B1 VS dist
%---------------------------------------------------% % Putting pedestrian positions in a separate vector % %---------------------------------------------------%
for i = 1:m
    x(i,1) = u((2*i)-1+(2*m));
    x(i,2) = u((2*i)+(2*m));
end %-------------------------------------------------------% % Setting inital position for average speed calculation % %-------------------------------------------------------%
if (t == 0)
    x0 = x;
end

%-------------------------------%
% Desired Velocity from (2.6.6) %
%-------------------------------%
for i = 1:m
if t == 0
v0a(i) = v0a0;
else
V (i,:) = x(i,:) - x0(i,:);
Vbar = norm(V(i,:)/t);
na = 1 - (Vbar/v0a0);
v0a(i) = (1 - na)*v0a0 + na*vmax;
end
%-----------------------%
% Desired destination p %
%-----------------------%
if (way == 1)
% Pedestrians in area A (L-Hand corner for Scramble Crossing) %
if (i <= m/2)
nrmpx(i) = norm(p(i,:) - x(i,:));
if (nrmpx(i) < dist)
D(i) = 1;
end
if (D(i) == 1)
p(i,:) = p1(1,:);
nrmpx(i) = norm(p(i,:) - x(i,:));
for j = 1:length(p1)
if (norm(p1(j,:) - x(i,:)) < nrmpx(i))
p(i,:) = p1(j,:);
nrmpx(i) = norm(p(i,:) - x(i,:));
end
end
end
% Pedestrians in area B (R-Hand corner for Scramble Crossing) %
elseif (i > m/2)
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
elseif (way == 0)
% Pedestrians in area A (L-Hand corner for Scramble Crossing) %
if (i <= m/2)
p(i,:) = p1(1,:);
nrmpx(i) = norm(p(i,:) - x(i,:));
for j = 1:length(p1)
if (norm(p1(j,:) - x(i,:)) < nrmpx(i))
p(i,:) = p1(j,:);
nrmpx(i) = norm(p(i,:) - x(i,:));
end

end
% Pedestrians in area B (R-Hand corner for Scramble Crossing) %
elseif (i > m/2)
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
if (nrmpx(i) == 0)
e(i,:) = [0,0];
else
e(i,:) = (p(i,:) - x(i,:))/(nrmpx(i));
end
%--------------------------------------%
% Driving Force component from (2.6.4) %
%--------------------------------------%
f((2*i)-1) = ((v0a(i)/tau) * e(i,1)) - (u((2*i)-1)/tau);
f((2*i)) = ((v0a(i)/tau) * e(i,2)) - (u((2*i))/tau);
%---------------------------------------------------------------------%
% Pedestrian Interactions from (2.6.8) - Summing all the interactions %
%---------------------------------------------------------------------%
fab = [0,0];
for j = 1:m
if (i ~= j)
dab = norm(x(i,:) - x(j,:));
if (dab < VS)
rab = r(i) + r(j);
nab = ((x(i,:) - x(j,:))/dab);
fab = fab + ((A1a*exp((rab - dab)/B1a))*nab)...
*(lambda + (1 - lambda)*((1+(-nab*e(i,:)'))/2))...
+ ((A2a*exp((rab - dab)/B2a))*nab);
end
end
end
f((2*i)-1) = f((2*i)-1) + fab(1,1);
f((2*i)) = f((2*i)) + fab(1,2);
%---------------------------------------------------------------------%
% Boundary Interaction component, this is analogous to the pedestrian %
% interactions - infact the formula only needs different constants %
%---------------------------------------------------------------------%
%-----------------------------------------------------%
% Summing all the boundary interactions from (2.6.12) %
%-----------------------------------------------------%
if BDRY == 1
for j = 1:length(bdry)
if (j == 1)
daB = norm(x(i,:) - bdry(j,:));
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
%----------------------------------%
% Displacement update from (2.6.1) %
%----------------------------------%
X((2*i)-1)= u((2*i)-1);
X(2*i) = u(2*i);
end
f0a = [f,X]';