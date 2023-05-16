%% provide desired end effector pose here
T06 =[     0.000000,    -1.000000,    -0.000000,   109.300000 ;
      0.707107,     0.000000,    -0.707107,  -402.520535 ;
      0.707107,     0.000000,     0.707107,   782.723800 ;
      0.000000,     0.000000,     0.000000,     1.000000 ];



%%defined DV hartenberg parameters plus 
%A = alpha, a = a, D = d

A1 = 0;
A2 = pi/2;
A3 = 0;
A4 = 0;
A5 = -pi/2;
A6 = pi/2;
 
a1 = 0;
a2 = 0;
a3 = 425;
a4 = 392.25;
a5 = 0;
a6 = 0;

D1=89.200;
D2=0;
D3=0;
D4=109.300;
D5=94.750;
D6=82.500;


%% vector from the provided end effector pose

P06 = [T06(1,4), T06(2,4), T06(3,4)];
X06 = [T06(1,1), T06(2,1), T06(3,1)];
Y06 = [T06(1,2), T06(2,2), T06(3,2)];
Z06 = [T06(1,3), T06(2,3), T06(3,3)];

%% calculation of theta_1




P05 = P06-D6*Z06;

phi1 = vpa(atan2(P05(2),P05(1)));

phi2 = vpa(acos((D4)/sqrt(P05(1)^2+P05(2)^2)));

theta_1 = vpa((phi1 + phi2 + pi/2));

theta_1degrees = round(vpa(rad2deg(theta_1)),2);

%% calculating theta5

%Solving for theta1
theta_5=vpa(-acos((sin(theta_1)*P06(1)-cos(theta_1)*P06(2)-D4)/D6));
theta_5degrees = round(vpa(rad2deg(theta_5)),2);

%% Calculating theta6

T60 = inv(T06);

X60x = T60(1,1);
X60y = T60(2,1);
Y60x = T60(1,2);
Y60y = T60(2,2);




theta_6 = vpa(atan2(-( sin(theta_1) * X60y + cos(theta_1) * Y60y)/sin(theta_5), (sin(theta_1) * X60x - cos(theta_1) * Y60x) / sin(theta_5)));
theta_6degrees = round(vpa(rad2deg(theta_6)),2);



%% theta 3

% løsning med første theta 1 værdi

% her bruges funktione TDH til at konstruere matricer 
T01 = TDH(A1,a1,D1,theta_1);
T45 = TDH(A5, a5, D5, theta_5);
T56 = TDH(A6, a6, D6, theta_6+pi);
T16 = inv(T01)* T06;
T14 =T16*inv(T45*T56);
T14Positionvector = T14(:,end);



lengthT14XZ= vpa((sqrt(T14Positionvector(1)^2+T14Positionvector(3)^2)));


theta_3=vpa(acos((lengthT14XZ^2-a3^2-a4^2)/(2*a3*a4)));
theta_3degrees = round((rad2deg(theta_3)),2);


%% Calculating Theta 2 


theta_2=atan2(-(T14Positionvector(3)),-(T14Positionvector(1)))- asin(a4*sin(theta_3)/sqrt(T14Positionvector(1)^2+T14Positionvector(3)^2));
theta_2=[theta_2];
theta_2deg = round(vpa(rad2deg(theta_2)),2);

%% Calculating theta 4

T12 = TDH(A2, a2, D2, theta_2+pi);

T23 = TDH(A3, a3, D3, theta_3);

T34 = inv(T12*T23)*(T14);


theta_4 =vpa(atan2(T34(2,1),T34(1,1)));

theta_4degrees=round(vpa(rad2deg(theta_4)),2);


%% all sols
Thetasoloution2 = [theta_1degrees, theta_2deg,  theta_3degrees, theta_4degrees ,theta_5degrees, theta_6degrees]
