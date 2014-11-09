%Fab
phi=(0:.1:2*pi);
lambda=0.3;




for i=1:length(phi);

fab(i)=(lambda + (1 - lambda)*((1+cos(phi(i)))/2));

end

length(fab)

plot(phi(:),fab(:))
