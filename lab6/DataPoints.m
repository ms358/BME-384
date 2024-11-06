%Define data points
xx=[2 3 5 7 9 10 9 12 6 3 11 1 5];yy=[2 4.3 2.5 3 5.5 7 4.5 6 5 1.5 3 2 6];zz=[4 6 5 6 6 7 7 7 4 3 5 3 4];
xx=xx';yy=yy';zz=zz';data=[xx,yy,zz];
xx1=xx+2;yy1=yy-2;zz1=zz-2;
xx2=xx1+[+1 -2 +1 -2 +1 -2 +1 -2 +1 -2 +1 -2 +1]';yy2=yy1+[+2 -1 +2 -1 +2 -1 +2 -1 +2 -1 +2 -1 +2]';

%plot the data
figure(1)
h1=plot3(xx,yy,zz,'ro','MarkerFace','r');hold on;
xlabel('x');ylabel('y');zlabel('z');view([0 0 1]);grid on;axis equal
plot3(mean(xx),mean(yy),mean(zz),'og','MarkerSize',10,'MarkerFace','g')
Target1=[4,3,2];Target2=[10,1,6];
plot3(Target1(1),Target1(2),Target1(3),"om")
plot3(Target2(1),Target2(2),Target2(3),"om")

h11=plot3(xx1,yy1,zz1,'bo','MarkerFace','b');hold on;
plot3(mean(xx1),mean(yy1),mean(zz1),'og','MarkerSize',10,'MarkerFace','g')

% 1. Absolute Constant Error:
%    Calculate the average distance from the target
absconsterror1 = ((Target1(1)-mean(xx))^2 + (Target1(2)-mean(yy))^2 + (Target1(3)-mean(zz))^2).^(1/2)
absconsterror2 = ((Target2(1)-mean(xx1))^2 + (Target2(2)-mean(yy1))^2 + (Target2(3)-mean(zz1))^2).^(1/2)

% 2. Constant Error:
%    Estimate the systematic shift from the target
%    by calculating the distance between the target and
%    the center of the cloud
% 3. Variable Error:
%    Estimate the consistency or reaching by
%    calculating the standard deviations of X, Y and Z coordinates
%    of the circles, and then calculating the square root of sum of squares of
%    these three STDs
%4.  Plot a sphere centerd at the center of the cloud, with the radius 
%    equal to the root mean square of std(x), std(y) and std(z). 
%5.  Plot an ellipsoid, rotate it using the first two eigenvectors 
%    in the COEFF matrix from the pca output 
%Bonus: 
%    Scale the axes of the ellipsoid using the pca output