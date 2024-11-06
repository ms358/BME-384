function [fingertip, Time, V1, V2, target] = phonk(file, label1, label2, label3, label4, fingertipmarkernumber, V1parameter, V2parameter, label5, targetmarkernumber)
data = readtable(file);
suckage = false;
global subplot_position;
switch file
    case "Data 2/Sahmet1A.csv"
        data(1:4,:) = [];
        data(:,3:26) = [];
        data(:,15:17) = [];
        V1parameter = 1;
        V2parameter = 1;
    case "Data 2/Sahmet1B.csv"
        data(1:4,:) = [];
        data(:,3:26) = [];
        V1parameter = 2;
        V2parameter = 2;
    case "Data 2/Sahmet2C.csv"
        data(1:4,:) = [];
        data(:,3:26) = [];
        data(:,18:20) = [];
        V1parameter = 3;
        V2parameter = 3;
        suckage = true;
    case "Data 2/Sahmet2DTarget1.csv"
        data(1:4,:) = [];
        data(:,3:26) = [];
        data(:,18:20) = [];
        V1parameter = 4;
        V2parameter = 4;
        suckage = true;
    case "Data 2/Sahmet2DTarget2.csv"
        data(1:4,:) = [];
        data(:,3:26) = [];
        V1parameter = 5;
        V2parameter = 5;
        suckage = true;
end
if suckage == true
    Position = table2array(data);
    Time = Position(:,2);
    X1 = Position(:,3);
    Y1 = Position(:,4);
    Z1 = Position(:,5);
    X2 = Position(:,6);
    Y2 = Position(:,7);
    Z2 = Position(:,8);
    X3 = Position(:,9);
    Y3 = Position(:,10);
    Z3 = Position(:,11);
    X4 = Position(:,12);
    Y4 = Position(:,13);
    Z4 = Position(:,14);
    X5 = Position(:,15);
    Y5 = Position(:,16);
    Z5 = Position(:,17);
    figure(subplot_position)
    plot3(X1,Y1,Z1); %Target
    hold on
    plot3(X2,Y2,Z2);
    plot3(X3,Y3,Z3);
    plot3(X4,Y4,Z4);
    plot3(X5,Y5,Z5);
    legend(label1, label2, label3, label4, label5)
    hold off

    % returns parameter for fingertip position
    switch fingertipmarkernumber
        case 1
            fingertip = [X1,Y1,Z1];%Target
        case 2
            fingertip = [X2,Y2,Z2];%Target
        case 3
            fingertip = [X3,Y3,Z3];%Target
        case 4
            fingertip = [X4,Y4,Z4];%Target
        case 5
            fingertip = [X5,Y5,Z5];%Target
        case 6
            fingertip = [X6,Y6,Z6];%Target
    end

    % returns extra parameter for target position
    switch targetmarkernumber
        case 1
            target = [X1,Y1,Z1];%Target
        case 2
            target = [X2,Y2,Z2];%Target
        case 3
            target = [X3,Y3,Z3];%Target
        case 4
            target = [X4,Y4,Z4];%Target
        case 5
            target = [X5,Y5,Z5];%Target
        case 6
            target = [X6,Y6,Z6];%Target
    end
else
    Position = table2array(data);
    Time = Position(:,2);
    X1 = Position(:,3);
    Y1 = Position(:,4);
    Z1 = Position(:,5);
    X2 = Position(:,6);
    Y2 = Position(:,7);
    Z2 = Position(:,8);
    X3 = Position(:,9);
    Y3 = Position(:,10);
    Z3 = Position(:,11);
    X4 = Position(:,12);
    Y4 = Position(:,13);
    Z4 = Position(:,14);
    figure
    plot3(X1,Y1,Z1); %wrist
    hold on
    plot3(X2,Y2,Z2); %fingertip
    plot3(X3,Y3,Z3); %shoulder
    plot3(X4,Y4,Z4); %elbow
    legend(label1, label2, label3, label4)
    hold off
    % returns parameter for fingertip position
    switch fingertipmarkernumber
        case 1
            fingertip = [X1,Y1,Z1];%Target
        case 2
            fingertip = [X2,Y2,Z2];%Target
        case 3
            fingertip = [X3,Y3,Z3];%Target
        case 4
            fingertip = [X4,Y4,Z4];%Target
        case 5
            fingertip = [X5,Y5,Z5];%Target
        case 6
            fingertip = [X6,Y6,Z6];%Target
    end
end
figure(6)
subplot(3, 2, subplot_position)
plot3(fingertip(:,1),fingertip(:,2),fingertip(:,3))
if suckage == true
    hold on
    plot3(target(:,1),target(:,2),target(:,3))
end
title("Condition " + subplot_position)
legend("Fingertip" + subplot_position)
if suckage == true
    legend("Target")
end
hold on
% returns V1 shoulder to elbow (manual because im not very good at this kinda thing🫥)
switch V1parameter
    case 1
        V1 = [X3 - X4, Y3 - Y4, Z3 - Z4];
    case 2
        V1 = [X4-X1, Y4-Y1, Z4-Z1];
    case 3
        V1 = [X2-X1, Y2-Y1, Z2-Z1];
    case 4
        V1 = [X2-X1, Y2-Y1, Z2-Z1];
    case 5
        V1 = [X1-X2,Y1-Y2, Z1-Z2];
end
% returns V2 elbow to wrist
switch V2parameter
    case 1
        V2 = [X4-X1, Y4-Y1, Z4-Z1];
    case 2
        V2 = [X1-X2, Y1-Y2, Z1-Z2];
    case 3
        V2 = [X2-X1, Y2-Y1, Z2-Z1];
    case 4
        V2 = [X1-X5, Y1-Y5, Z1-Z5];
    case 5
        V2 = [X2-X4, Y2-Y4, Z2-Z4];
end

subplot_position = subplot_position + 1;