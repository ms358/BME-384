load data2.mat
Target=[10,12,7];
plot3(Target(1),Target(2),Target(3),'bo','MarkerSize',10,'MarkerFaceColor','b')
hold on
h1=plot3(xx,yy,zz,'ro','MarkerFace','r');hold on;xlabel('X');ylabel('Y');zlabel('Z');hold on
plot3(mean(xx),mean(yy),mean(zz),'og','MarkerSize',10,'MarkerFace','g')
axis equal
% 1. Absolute Constant Error:
%    Calculate the average distance from the target
% 2. Constant Error:
%    Estimate the systematic shift from the target
%    by calculating the distance between the target and
%    the center of the cloud
% 3. Variable Error:
%    Estimate the consistency or reaching by
%    calculating the standard deviations of X, Y and Z coordinates
%    of the circles, and then calculating the square root of sum of squares of
%    these three STDs
%4.  Plot a sphere centered at the center of the cloud, with the radius 
%    equal to the square root of the sum of squares of std(x), std(y), and std(z). 
%5.  Plot an ellipsoid, rotate it using the first two eigenvectors 
%    from the matrix COEFF in the PCA output 
%Bonus: 
%    Scale the axes of the ellipsoid using the eigenvalues provided by the PCA output

[x, y, z] = ellipsoid(mean(xx),mean(yy),mean(zz),2,1,3,30);
h3=surf(x, y, z);
h3.FaceAlpha=.01;
set(h3,'LineStyle',':');
axis equal,grid on;view([0 0 1]);

%First eigenvector:
data=[xx-mean(xx);yy-mean(yy);zz-mean(zz)];data=data';
[COEFF, SCORE, LATENT, TSQUARED, EXPLAINED] = pca(data);
line([mean(xx) (mean(xx)+5*COEFF(1,1))],[mean(yy) (mean(yy)+5*COEFF(1,2))],[mean(zz) (mean(zz)+5*COEFF(1,3))],'LineWidth',3)
line([mean(xx) mean(xx)+5],[mean(yy) mean(yy)],[mean(zz) mean(zz)],'LineWidth',3)
alpha=atand(COEFF(1,2)/COEFF(1,1))
rotate(h3,[0 0 1],alpha,[mean(xx) mean(yy) mean(zz)])
beta=atand(COEFF(1,1)/COEFF(1,3))
rotate(h3,[0 1 0],beta,[mean(xx) mean(yy) mean(zz)])

%Second eigenvector: