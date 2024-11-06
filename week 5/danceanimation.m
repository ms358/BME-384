clc;
clear variables;
close all;
data1 = readtable("Dance1.csv");
data1array = table2array(data1);
Time=data1array(:,1);
X1=data1array(:,2);
Y1=data1array(:,3);
Z1=data1array(:,4);
X2=data1array(:,5);
Y2=data1array(:,6);
Z2=data1array(:,7);
X3=data1array(:,8);
Y3=data1array(:,9);
Z3=data1array(:,10);
X4=data1array(:,11);
Y4=data1array(:,12);
Z4=data1array(:,13);
X5=data1array(:,14);
Y5=data1array(:,15);
Z5=data1array(:,16);
X6=data1array(:,17);
Y6=data1array(:,18);
Z6=data1array(:,19);
X7=data1array(:,20);
Y7=data1array(:,21);
Z7=data1array(:,22);
X8=data1array(:,23);
Y8=data1array(:,24);
Z8=data1array(:,25);
X9=data1array(:,26);
Y9=data1array(:,27);
Z9=data1array(:,28);
X10=data1array(:,29);
Y10=data1array(:,30);
Z10=data1array(:,31);
X11=data1array(:,32);
Y11=data1array(:,33);
Z11=data1array(:,34);
figure
left_knee = plot3(X1,Y1,Z1,'o');hold on
right_hand = plot3(X2,Y2,Z2,'o');
right_shoulder = plot3(X3,Y3,Z3,'o');
right_foot = plot3(X4,Y4,Z4,'o');
right_elbow = plot3(X5,Y5,Z5,'o');
left_hand = plot3(X6,Y6,Z6,'o');
right_knee = plot3(X7,Y7,Z7,'o');
left_shoulder = plot3(X8,Y8,Z8,'o');
left_elbow = plot3(X9,Y9,Z9,'o');
left_hip = plot3(X10,Y10,Z10,'o');
right_hip = plot3(X11,Y11,Z11,'o');
axis([-1.5 1.5 0 3.5 -5 2]);
view(0, 90);
rightforearm=line([X2(1), X5(1)], [Y2(1), Y5(1)], [Z2(1), Z5(1)], 'Color', 'b');
rightupperarm=line([X5(1), X3(1)], [Y5(1), Y3(1)], [Z5(1), Z3(1)], 'Color', 'r');
leftforearm = line([X6(1), X9(1)], [Y6(1), Y9(1)], [Z6(1), Z9(1)], 'Color', 'b');
leftupperarm = line([X9(1), X8(1)], [Y9(1), Y8(1)], [Z9(1), Z8(1)], 'Color', 'b');
rightthigh = line([X11(1), X7(1)], [Y11(1), Y7(1)], [Z11(1), Z7(1)], 'Color', 'b');
rightshin = line([X7(1), X4(1)], [Y7(1), Y4(1)], [Z7(1), Z4(1)], 'Color', 'b');
leftthigh = line([X10(1), X1(1)], [Y10(1), Y1(1)], [Z10(1), Z1(1)], 'Color', 'b');
hips = line([X10(1), X11(1)], [Y10(1), Y11(1)], [Z10(1), Z11(1)], 'Color', 'b');
shoulders = line([X8(1), X3(1)], [Y8(1), Y3(1)], [Z8(1), Z3(1)], 'Color', 'b');

for l = 1:length(Time)
    %draw points :D
    set(left_knee, 'XData', X1(l),'YData', Y1(l),'ZData', Z1(l))
    set(right_hand, 'XData', X2(l),'YData', Y2(l),'ZData', Z2(l))
    set(right_shoulder, 'XData', X3(l),'YData', Y3(l),'ZData', Z3(l))
    set(right_foot, 'XData', X4(l),'YData', Y4(l),'ZData', Z4(l), "Color","r")
    set(right_elbow, 'XData', X5(l),'YData', Y5(l),'ZData', Z5(l))
    set(left_hand, 'XData', X6(l),'YData', Y6(l),'ZData', Z6(l))
    set(right_knee, 'XData', X7(l),'YData', Y7(l),'ZData', Z7(l))
    set(left_shoulder, 'XData', X8(l),'YData', Y8(l),'ZData', Z8(l))
    set(left_elbow, 'XData', X9(l),'YData', Y9(l),'ZData', Z9(l))
    set(left_hip, 'XData', X10(l),'YData', Y10(l),'ZData', Z10(l))
    set(right_hip, 'XData', X11(l),'YData', Y11(l),'ZData', Z11(l))
    %draw lines :D
    set(rightforearm, 'XData', [X2(l), X5(l)], 'YData', [Y2(l), Y5(l)], 'ZData', [Z2(l), Z5(l)]);
    set(rightupperarm, 'XData', [X5(l), X3(l)], 'YData', [Y5(l), Y3(l)], 'ZData', [Z5(l), Z3(l)]);
    set(leftforearm, 'XData', [X6(l), X9(l)], 'YData', [Y6(l), Y9(l)], 'ZData', [Z6(l), Z9(l)]);
    set(leftupperarm, 'XData', [X9(l), X8(l)], 'YData', [Y9(l), Y8(l)], 'ZData', [Z9(l), Z8(l)]);
    set(rightthigh, 'XData', [X11(l), X7(l)], 'YData', [Y11(l), Y7(l)], 'ZData', [Z11(l), Z7(l)]);
    set(rightshin, 'XData', [X7(l), X4(l)], 'YData', [Y7(l), Y4(l)], 'ZData', [Z7(l), Z4(l)]);%
    set(leftthigh, 'XData', [X10(l), X1(l)], 'YData', [Y10(l), Y1(l)], 'ZData', [Z10(l), Z1(l)]);%
    set(hips, 'XData', [X10(l), X11(l)], 'YData', [Y10(l), Y11(l)], 'ZData', [Z10(l), Z11(l)]);%
    set(shoulders, 'XData', [X8(l), X3(l)], 'YData', [Y8(l), Y3(l)], 'ZData', [Z8(l), Z3(l)]);
    drawnow
    %pause(0.01)
end