% Slam_A-[GANO Group]

%HMT:    "How many time", to count how many landmarks it has seen.
%Zest:   Estimated state of robot's pose and landmarks' positions
%Pest:   Diagonal of Covariance matrix of estimated state
%Ptotal: Covariance matrix of estimated state

%% Loading
clear all 
close all 

%% SETUP DATASET
exercise='A';

%Change the number in this string to change the dataset of simulation
dataset="data_point_land_1.mat";

data_number=sscanf(dataset,'data_point_land_%d.mat');
load(dataset);
% Horizon time
T = [1:Ts:(size(Ua)+1)*Ts]; 

%% Initialization State
% Initial robot's state
z_u0 = Pose(1,:)'; 
% Initial robot's landmarks
z_l0 = zeros(40,1);
% Initial state
z0 = [z_u0;     
      z_l0];

%% Initialization Covariance Matrix
%Tuning parameter for robot's state
lambda = [0.0001 0.0001 0.0001]';
%Covariance matrix robot's state
P_u0 = lambda.*eye(3);
%Tuning parameter for landmark's state
eta = 100;
%Covariance matrix landmark's state
P_l0 = eta*eye(2*Nland);
%Covariance matrix landmarks' state
P_0 = blkdiag(P_u0, P_l0);

%-----------------------------------------------------------


%% Auxiliary parameters for EKF Algorithm
N = size(Ua,1); 
Pp = P_0; 
Zp = z0;
Qs = zeros(2,2);
HMT=zeros(N,20);
CheckL=zeros(1,20);

%% EKF Algorithm
for t=1:N
    disp(['t: ', num2str(t), ', Landmarks: ', num2str((size(Zp,1)-3)/2)]);
    if t~=1
     HMT(t,:)=HMT(t-1,:);
    end
    % Correction step
    for i=1:size(Meas.land{t},1)
        %Landmarks' label
        L = Meas.land{t}(i); 
        % Check landmark seen first time
        if CheckL(L)==0
         CheckL(L)=1;
         HMT(t,L)=HMT(t,L)+1;
         % Absolute components of landmark
         Zp(2+2*L) = Meas.range{t}(i)*cos(wrapToPi(Meas.angle{t}(i))+Zp(3))+Zp(1);
         Zp(3+2*L) = Meas.range{t}(i)*sin(wrapToPi(Meas.angle{t}(i)+Zp(3)))+Zp(2);
         % Relative components of landmark
         dx=Zp(2+2*L)-Zp(1);
         dy=Zp(3+2*L)-Zp(2);
         q=sqrt(dx^2+dy^2);        
        else
         HMT(t,L)=HMT(t,L)+1;
         % Auxiliary parameters for computation of H
         dx = Zp(2+2*L)-Zp(1);
         dy = Zp(3+2*L)-Zp(2);
         q = sqrt(dx^2+dy^2);
        end

        % H Jacobian Matrix computation
        H=zeros(2,3+2*Nland);
        H(:,1:3) = [-dx/q -dy/q 0; 
                    dy/q^2 -dx/q^2 -1];
        H(:,2+2*L:3+2*L) = [dx/q dy/q; -dy/q^2 dx/q^2];

        % Kalman Gain
        K = Pp*H'*inv(H*Pp*H'+R); 
        % Correction step
        Zp = Zp+K*([Meas.range{t}(i)-q;wrapToPi(Meas.angle{t}(i)-(atan2(dy,dx)-Zp(3)))]); 
        Pp = Pp*(eye(43)-H'*K');   
    end 
    Zc=Zp;
    Pc=Pp;
    % Prediction Step
        % F Jacobian Matrix computation
        F_u = [1  0  -Ts*Uf(t)*sin(Zc(3));
               0  1   Ts*Uf(t)*cos(Zc(3));
               0  0     1];
        Z=blkdiag(F_u,eye(40));
       
       %System evalution: robot
       Zp(1) = Zc(1) + Ts*Uf(t)*cos(Zc(3)); 
       Zp(2) = Zc(2) + Ts*Uf(t)*sin(Zc(3));
       Zp(3) = Zc(3) + Ts*Ua(t); 

       %System evalution: landmarks
       for i= 1:size(Meas.land{t})
           L = Meas.land{t}(i); 
           Zp(2+2*L) = Zc(2+2*L); 
           Zp(3+2*L) = Zc(3+2*L);
       end 
       
       % G Jacobian Matrix computation
        G_u=[-Ts*cos(Zc(3))  0;
             -Ts*sin(Zc(3))  0;
                0        -Ts]; 
        G = [G_u; zeros(Nland*2,2)]; 
       
      % Process disturbance evaluation
       if abs(Ua(t)) > wturn
           Qs = Qturn; 
       else
           Qs = Q; 
       end 
       % Computation covariance matrix prediction error
       Pp = Z*Pc*Z'+ G*Qs*G'; 

       % Saving data of interest
       Ptotal(t,:)=reshape(Pc.',1,[]);
       Zest(t,:) = Zc'; 
       Pest(t,:) = diag(Pc)'; 
end 
%Trajectory by integration
Zi(1,:)=[1,1,0];

for t=1:220
 Zi(1) = Zi(1) + Ts*Uf(t)*cos(Zi(3)); 
 Zi(2) = Zi(2) + Ts*Uf(t)*sin(Zi(3));
 Zi(3) = Zi(3) + Ts*Ua(t);
 
 Ziest(t,:)=Zi;
end

SLAM_A_Data_Plot;

 
function [Zp,Pp]=CorrectionStep(L,Zp,H,Pp,R,MeasRange,MeasAngle)
  
    [dx,dy,q]=differential(Zp,L);
    
    % H Jacobian Matrix computation
    H=zeros(2,size(H,2));
    H(:,1:3) = [-dx/q -dy/q 0;
        dy/q^2 -dx/q^2 -1];
    H(:,2+2*L:3+2*L) = [dx/q dy/q; -dy/q^2 dx/q^2];
    
    % Kalman Gain
    K = Pp*H'*inv(H*Pp*H'+R);
    % Correction step
    Zp = Zp+K*([MeasRange-q;wrapToPi(MeasAngle-(atan2(dy,dx)-Zp(3)))]);
    Pp = Pp*(eye(size(Pp,1))-H'*K');
end



