clear;

%% Example values from Taebaek

% Stereo calibration result (stereo_rgb.yaml)
% 
% %YAML:1.0
% K_rgb1: !!opencv-matrix
%   rows: 3
%   cols: 3
%   dt: d
%   data: [   762.91936652,      0.88235488,    638.01009524,      0.00000000,    770.25435429,    499.48141012,      0.00000000,      0.00000000,      1.00000000]
% R_rgb1: !!opencv-matrix
%   rows: 3
%   cols: 3
%   dt: d
%   data: [     1.00000000,      0.00000000,      0.00000000,      0.00000000,      1.00000000,      0.00000000,      0.00000000,      0.00000000,      1.00000000]
% T_rgb1: !!opencv-matrix
%   rows: 3
%   cols: 1
%   dt: d
%   data: [     0.00000000,      0.00000000,      0.00000000]
% D_rgb1: !!opencv-matrix
%   rows: 5
%   cols: 1
%   dt: d
%   data: [    -0.03701832,      0.10225075,      0.00028546,     -0.00016501,     -0.05264264]
% 
% K_rgb2: !!opencv-matrix
%   rows: 3
%   cols: 3
%   dt: d
%   data: [   761.75432126,     -1.10063624,    636.39807791,      0.00000000,    768.27087821,    503.93565399,      0.00000000,      0.00000000,      1.00000000]
% R_rgb2: !!opencv-matrix
%   rows: 3
%   cols: 3
%   dt: d
%   data: [     0.99971079,     -0.02403828,      0.00070129,      0.02404379,      0.99966729,     -0.00933877,     -0.00047656,      0.00935293,      0.99995615]
% T_rgb2: !!opencv-matrix
%   rows: 3
%   cols: 1
%   dt: d
%   data: [  -473.82652643,     -5.20784313,     -0.45308639]
% D_rgb2: !!opencv-matrix
%   rows: 5
%   cols: 1
%   dt: d
%   data: [    -0.04728323,      0.11789207,      0.00182861,     -0.00019081,     -0.06224091]

% Kalibr camera-IMU calibration result
% (results-imucam-homercvlabDatasetzzangys2020-12-04-14-19-36.txt)
% 
% Calibration results
% ===================
% Reprojection error squarred (cam0):  mean 0.286270230004, median 0.121186404316, std: 0.515249337666
% Gyro error squarred (imu0):          mean 1.63520282863, median 0.919310783343, std: 2.09361505517
% Accelerometer error squarred (imu0): mean 0.10321840415, median 0.0587283469493, std: 0.139837404505
% 
% Transformation (cam0):
% -----------------------
% T_ci:  (imu to cam0): [m]
% [[ 0.05632227 -0.99837924  0.00816638 -0.00481624]
%  [-0.00232928 -0.00831074 -0.99996275  0.00389919]
%  [ 0.99840992  0.05630115 -0.00279358 -0.06663181]
%  [ 0.          0.          0.          1.        ]]
% 
% T_ic:  (cam0 to imu): [m]
% [[ 0.05632227 -0.00232928  0.99840992  0.0668062 ]
%  [-0.99837924 -0.00831074  0.05630115 -0.00102459]
%  [ 0.00816638 -0.99996275 -0.00279358  0.00375223]
%  [ 0.          0.          0.          1.        ]]
% 
% timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)
% 0.0
% 
% 
% Gravity vector in target coords: : [m/s^2]
% [ 0.05625547 -8.45163731 -4.98023716]
% 
% 
% Calibration configuration
% =========================
% 
% cam0
% -----
%   Camera model: pinhole
%   Focal length: [759.37924, 760.37223]
%   Principal point: [638.72186, 522.12229]
%   Distortion model: radtan
%   Distortion coefficients: [-0.02988, 0.04773, 0.00014, 0.0002]
%   Type: aprilgrid
%   Tags: 
%     Rows: 6
%     Cols: 6
%     Size: 0.05125 [m]
%     Spacing 0.01549999998 [m]
% 
% 
% 
% IMU configuration
% =================
% 
%   Update rate: 100.0
%   Accelerometer:
%     Noise density: 0.06 
%     Noise density (discrete): 0.6 
%     Random walk: 0.6
%   Gyroscope:
%     Noise density: 0.007
%     Noise density (discrete): 0.07 
%     Random walk: 0.07


%% Copy calibration results from yaml/txt files

% Stereo extrinsics
R_rgb2 = [     0.99971079,     -0.02403828,      0.00070129,      0.02404379,      0.99966729,     -0.00933877,     -0.00047656,      0.00935293,      0.99995615];
T_rgb2 = [  -473.82652643,     -5.20784313,     -0.45308639];

% Camera-IMU extrinsics
T_ic = [[ 0.05632227 -0.00232928  0.99840992  0.0668062 ]
 [-0.99837924 -0.00831074  0.05630115 -0.00102459]
 [ 0.00816638 -0.99996275 -0.00279358  0.00375223]
 [ 0.          0.          0.          1.        ]];


%% Calculate transforms
% R_rgb2 = [r11, r12, r13, r21, r22, r23, r31, r32, r33]
% -> [r11, r12, r13; r21, r22, r23; r31, r32, r33]
R_rgb2 = reshape(R_rgb2, [3, 3])';
% T_rgb2 : mm to m
T_rgb2 = T_rgb2' / 1000.0;

% Point transform from rgb1 to rgb2
P_rgb2 = [R_rgb2, T_rgb2; 0, 0, 0, 1];

% Coordinate transform from rgb1 to rgb2
cam0_T_cam1 = inv(P_rgb2);

% Copy Kalibr result
body_T_cam0 = T_ic;

% body_T_cam1 = body_T_cam0 * cam0_T_cam1
body_T_cam1 = body_T_cam0 * cam0_T_cam1;

% Display result
format LONG

fprintf(1, 'Calibration results (Copy to config) :\n\n');

fprintf(1, 'body_T_cam0 : \n\n');
fprintf(1, '[ %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam0(1, 1), body_T_cam0(1, 2), body_T_cam0(1, 3), body_T_cam0(1, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam0(2, 1), body_T_cam0(2, 2), body_T_cam0(2, 3), body_T_cam0(2, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam0(3, 1), body_T_cam0(3, 2), body_T_cam0(3, 3), body_T_cam0(3, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f]\n\n', body_T_cam0(4, 1), body_T_cam0(4, 2), body_T_cam0(4, 3), body_T_cam0(4, 4));

fprintf(1, 'body_T_cam1 : \n\n');
fprintf(1, '[ %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam1(1, 1), body_T_cam1(1, 2), body_T_cam1(1, 3), body_T_cam1(1, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam1(2, 1), body_T_cam1(2, 2), body_T_cam1(2, 3), body_T_cam1(2, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f,\n', body_T_cam1(3, 1), body_T_cam1(3, 2), body_T_cam1(3, 3), body_T_cam1(3, 4));
fprintf(1, '  %+18.15f, %+18.15f, %+18.15f, %+18.15f]\n\n', body_T_cam1(4, 1), body_T_cam1(4, 2), body_T_cam1(4, 3), body_T_cam1(4, 4));


