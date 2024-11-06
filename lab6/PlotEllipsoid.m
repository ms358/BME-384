close all;clear all;
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


h11=plot3(xx1,yy1,zz1,'bo','MarkerFace','b');hold on;
plot3(mean(xx1),mean(yy1),mean(zz1),'og','MarkerSize',10,'MarkerFace','g')

%subtract the mean and plot the data points again
figure(2)
XX=xx-mean(xx);YY=yy-mean(yy);ZZ=zz-mean(zz);DATA=[XX,YY,ZZ];
h2=plot3(XX,YY,ZZ,'ro','MarkerFace','r');
xlabel('X');ylabel('Y');zlabel('Z');view([0 0 1]);hold on;grid on;axis equal
h20=plot3(mean(XX),mean(YY),mean(ZZ),'og','MarkerSize',10,'MarkerFace','g');

%Principal Component Analysis (PCA)
A=cov(DATA);%covariance matrix
[V,D]=eig(A);%calculate eignevectors and eigenvalues
V1=V(:,3)*sqrt(D(3,3));V2=V(:,2)*sqrt(D(2,2));V3=V(:,1)*sqrt(D(1,1));
h21=line([V1(1) -V1(1)],[V1(2) -V1(2)],[V1(3) -V1(3)],'Color', 'r','LineWidth',3);
h22=line([V2(1) -V2(1)],[V2(2) -V2(2)],[V2(3) -V2(3)],'Color', 'g','LineWidth',3);
h23=line([V3(1) -V3(1)],[V3(2) -V3(2)],[V3(3) -V3(3)],'Color', 'b','LineWidth',3);

%fit an ellipsoid to the cloud of points

%plot unrotated ellipsoid with axis lengths equal to eigenvalues
[x, y, z] = ellipsoid(0,0,0,2*sqrt(D(3,3)),2*sqrt(D(2,2)),2*sqrt(D(1,1)),30);
h3=surf(x, y, z);
set(h3,'LineStyle',':');
h3.FaceAlpha=.1;
axis equal,grid on
%angle between the X axis and the first eigenvector V1
ALPHA1=acosd(dot(V1,[1 0 0])/norm(V1));
%angle between the Y axis and the second eigenvector V2
ALPHA2=acosd(dot(V2,[0 1 0])/norm(V1)*norm(V2));%angle between the X axis and the first eigenvector V1
%rotate the ellipsoid to align its largest axis with V1
rotate(h3,cross([1 0 0],V1),ALPHA1,[0 0 0])
%rotate the ellipsoid to align its second largest axis with V2
rotate(h3,cross(V2,V3),ALPHA2,[0 0 0])
axis equal

%plot the original points and translate the rotated ellipsoid to fit them  
h4=plot3(xx,yy,zz,'ro','MarkerFace','m');
h5=plot3(mean(xx),mean(yy),mean(zz),'og','MarkerSize',10,'MarkerFace','g');
Xdisp=h3.XData;Ydisp=h3.YData;Zdisp=h3.ZData;
h6=surf(Xdisp+mean(xx), Ydisp+mean(yy),Zdisp+mean(zz));
set(h6,'LineStyle',':');
h6.FaceAlpha=.1;

h31=line([V1(1)+mean(xx) -V1(1)+mean(xx)],[V1(2)+mean(yy) -V1(2)+mean(yy)],[V1(3)+mean(zz) -V1(3)+mean(zz)],'Color', 'r','LineWidth',3);
h32=line([V2(1)+mean(xx) -V2(1)+mean(xx)],[V2(2)+mean(yy) -V2(2)+mean(yy)],[V2(3)+mean(zz) -V2(3)+mean(zz)],'Color', 'g','LineWidth',3);
h33=line([V3(1)+mean(xx) -V3(1)+mean(xx)],[V3(2)+mean(yy) -V3(2)+mean(yy)],[V3(3)+mean(zz) -V3(3)+mean(zz)],'Color', 'b','LineWidth',3);

delete(h2);delete(h3);delete(h20);delete(h21);delete(h22);delete(h23)


% function h = circle(x,y,r)
% hold on
% th = 0:pi/50:2*pi;
% xunit = r * cos(th) + x;
% yunit = r * sin(th) + y;
% h = plot(xunit, yunit);
% hold off
