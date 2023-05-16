
%Denavit hartenberg paramters

L1=Link('d',  89.2,   'a',  0,    'alpha',   0,         'modified');
L2=Link('d',  0,      'a',  0,    'alpha',   90*pi/180,'offset', 180*pi/180, 'modified');
L3=Link('d',  0,      'a',  425,  'alpha',   0,         'modified');
L4=Link('d',  109.3,  'a',  392,  'alpha',   0,         'modified');
L5=Link('d',  94.75,  'a',  0,    'alpha',  -90*pi/180, 'modified');
L6=Link('d',  82.5,   'a',  0,    'alpha',   90*pi/180,'offset', 180*pi/180, 'modified');

% Linking the robot togheter
UR5 = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'UR5');

% Plotting the robot

