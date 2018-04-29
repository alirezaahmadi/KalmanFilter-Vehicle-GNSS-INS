%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Kalman Fliter Implementation on IMU+GNSS Recorded Data  %
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AhmadiAlireza.webs.com                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear all; close all;
%%***************definition of parameters************************
 a = 6378137.0;        %m Earth's ellipsoid radius at wquator
 b = 6356752.3142 ;    %m Earth's ellipsoid radius at poles
 ecc = 0.0818191908426;  %- Earth's ellipsoid eccentricity
 w_i_e = 7.2921150*10^-5;   %rad/s Earth's rotational speed
 mu = 3.986004418*10^14;  %m^3/s^2 Geocentric gravitional constant
 f = 1/298.257223563;  %- Earth's ellipsoid flattening
 omega_ib_b = zeros(3,3);
 g0 = 0;
 R2D = 180/pi;
 D2R = pi/180;
%%***************************************************************
%filename0  = 'IMAR0000.mat';
%filename1  = 'IMAR0001.mat';
filename2  = 'IMAR0002.mat';
%filename3  = 'IMAR0003.mat';

load(filename2,'-regexp','imu');
load(filename2,'-regexp','gps');

%filename00  = 'UTM_IMAR0000.mat';
%filename01  = 'UTM_IMAR0001.mat';
filename02  = 'UTM_IMAR0002.mat';
%filename03  = 'UTM_IMAR0003.mat';

load(filename02,'-regexp','UTM');


x_0 = UTM(1,1);
y_0 = UTM(1,2);
phi_0 = imu.rpy_ned(1,3);
phi_D_0 = 0;
v_0 = 0;
a_0 = 0;

x_i = 0;
y_i = 0;
phi_i = 0;
phi_D_i = 0;
v_i = 0;
a_i = 0;

Delta_t=0.001;

Variance_phi_D = 0.0035;
Varince_a = 0.01^2;

Variance_x = 10^-4;
Variance_y = 10^-4;
Variance_phi = 10^-5;

P_kM1 = [Variance_x,0,0,0,0,0;
       0,Variance_y,0,0,0,0;
       0,0,Variance_phi,0,0,0;
       0,0,0,0,0,0;
       0,0,0,0,0,0;
       0,0,0,0,0,0];
       
A = size(UTM);
Res = A(1,1);
B = size(imu);
cnt = 1;
cnt_b = 1;
Interval =1;
P_Interval =10;

for cnt=1:15:length(UTM)
P1 =plot(UTM(cnt,1),UTM(cnt,2),'r.');
hold on
end


cnt=Interval+1;
while  cnt<=Res
cnt;

%*****************Prediction*****************************
  x_kM = [x_0 + v_0*Delta_t*cos(phi_0);
          y_0 + v_0*Delta_t*sin(phi_0);
          phi_0 + phi_D_0*Delta_t;
          phi_D_0;
          v_0+a_0*Delta_t;
          a_0];
  
%  Ans1(cnt,:)=[x_kM(1,1),x_kM(2,1)];
%  hold on
%  if Ans1(cnt,1)~= 0
%  plot(Ans1(cnt,1),Ans1(cnt,2),'g');
%  end
          
  Tans_Matrix = [1,0,-v_0*Delta_t*sin(phi_0),0,Delta_t*cos(phi_0),0;
                 0,1, v_0*Delta_t*cos(phi_0),0,Delta_t*sin(phi_0),0;
                 0,0,1,Delta_t,0,0;
                 0,0,0,1,0,0;
                 0,0,0,0,1,Delta_t;
                 0,0,0,0,0,1];
                 
  Q = [Variance_phi_D,0;
       0,Varince_a];
       
  GQG_T = [0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,0;
           0,0,0,Variance_phi_D*Delta_t^2,0,0;
           0,0,0,0,0,0;
           0,0,0,0,0,Varince_a*Delta_t^2];
   
  P_k_M = (Tans_Matrix * P_kM1 * Tans_Matrix') + GQG_T;
  
%*****************Correction*****************************

%  Varince_x_gps = 0.05^2;
%  Varince_y_gps = 0.05^2;
%  Variance_Acc  = 0.001^2;
%  Variance_gyro = 5*10^-16;

  Varince_x_gps = 0.01;
  Varince_y_gps = 0.01;
  Variance_Acc  = 0.001;
  Variance_gyro = 5*10^-5;

 
  z_k = [UTM(cnt,1);
         UTM(cnt,2);
         -imu.acc_ib_b(cnt_b,1);
         imu.omg_ib_b(cnt_b,3)];
  
  R_gps = [Varince_x_gps,0,0,0;
           0,Varince_y_gps,0,0;
           0,0,Variance_Acc,0;
           0,0,0,Variance_gyro];
           
  H = [1,0,0,0,0,0;
       0,1,0,0,0,0;
       0,0,0,0,0,1;
       0,0,0,1,0,0];
   
  h = H * x_kM;

  K_k = P_k_M * H' * (inv(H * P_k_M * H' + R_gps));
  
  x_k = x_kM + K_k*(z_k - h);
  
  P_k = (eye(6) - K_k * H)* P_k_M;
  
  Ans(cnt,:)=[x_k(1,1),x_k(2,1)];
  hold on
  if Ans(cnt,1)~= 0
  P2=plot(Ans(cnt,1),Ans(cnt,2),'b');
  end
  
  P_kM1 = P_k_M;
  
  x_0 = x_k(1,1);
  y_0 = x_k(2,1);
  phi_0 = x_k(3,1);
  phi_D_0 = x_k(4,1);
  v_0 = x_k(5,1);
  a_0 = x_k(6,1);
  
  
  cnt = cnt+Interval;
  cnt_b = cnt_b + P_Interval;
   
  %Time fraction 
  Delta_t = (imu.imu_time(cnt_b,1) - imu.imu_time(cnt_b-P_Interval,1));
  
  
end
xlabel('UTM-East');
ylabel('UTM-North');
legend([P1,P2],'UTM','EKF');











