r = robot();

%translate(r, 50, 1); % translate robot
%delete(r); % stop robot
%rotate(r, 50, 90); % rotate robot
distances_m = survey(r, 50, -60) % take readings from ultra-sound sensor
tempUltraScan = [distances_m(4,2) distances_m(5,2) distances_m(6,2) distances_m(1,2) distances_m(2,2) distances_m(3,2)]


