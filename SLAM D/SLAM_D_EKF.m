% Slam project-V 2.4

%% Loading
clear all 
close all 

%% SETUP DATASET
exercise='D';

%CHANGE DATA SET
% "gruppo1_1_1.mat" : 1 SIDE EXPERIMENT 1
% "gruppo1_1_2.mat" : 1 SIDE EXPERIMENT 2
% "gruppo1_2_1.mat" : 4 SIDEs EXPERIMENT 1
% "gruppo1_2_2.mat" : 4 SIDEs EXPERIMENT 2


dataset="gruppo1_1_1.mat";
data_number=sscanf(dataset,'gruppo1_%d.mat');
load(dataset);


% Horizon time
Ts = diff(TimeStamp);

%% Initialization State 
% Initial robot's state %3x1+2x1
z0 = [41.5 59 pi]'; 
 
if dataset=="gruppo1_1_1.mat"
    % %% PARAMETER 1 SIDE-EXPERIMENT 1

    lambda = [0.002 0.002 0.00055]';
    P_0 = lambda.*eye(3);
    eta=180;

    sigma = 0.16;

    R = [0.01 0;
        0 0.0015];

    wturn = 0.23;

    Q = [sigma^2 0;
        0 0.1283];

    Qturn = [sigma^2 0;
        0 0.0767];

    tau1=4;
    tau2=24.107;
    Threshold=0.999;
    Prominence=0.320;

elseif  dataset=="gruppo1_1_2.mat"
    %% PARAMETERS 1 SIDE-EXPERIMENT 2

    lambda = [0.2 0.2 0.0003]';
    P_0 = lambda.*eye(3); %3x3 +eta*diag(eye)2x2
    eta=10;

    sigma = 0.0075;

    R = [0.095 0;
        0 0.056];

    wturn = 0.12;

    Q = [sigma^2 0;
        0 3.13e-5];

    Qturn = [sigma^2 0;
        0 0.1];

    tau1=4.5;
    tau2=27.507;
    Threshold=0.0335;
    Prominence=0.075;

elseif dataset=="gruppo1_2_1.mat"
    %% PARAMETERS 4 SIDE- EXPERIMENT 1

    lambda = [0.2 0.2 0.0002]';
    P_0 = lambda.*eye(3); %3x3 +eta*diag(eye)2x2
    eta=90;

    sigma = 0.015;

    R = [0.01 0;
        0 0.0015];

    wturn = 0.12;

    Q = [0.011^2 0;
        0 3.13e-6];

    Qturn = [sigma^2 0;
        0 0.031];

    tau1=2.5;
    tau2=27.507;
    Threshold=0.45;
    Prominence=0.18;

elseif dataset=="gruppo1_2_2.mat"
    %% PARAMETERS 4 SIDE-EXPERIMENT 2

    lambda = [0.01 0.01 0.0001]';
    P_0 = lambda.*eye(3); %3x3 +eta*diag(eye)2x2
    eta=10;

    sigma = 0.0067;

    R = [0.01 0;
        0 0.0015];

    wturn = 0.17;

    Q = [sigma^2 0;
        0 3.13e-6];

    Qturn = [sigma^2 0;
        0 0.045];

    tau1=4.5;
    tau2=22.507;
    Threshold=0.78;
    Prominence=0.21;
end

%--------------------------------------------------------------------------


%% Auxiliary parameters for EKF Algorithm
N = size(Ua,2)-1; 
Pp = P_0; 
Zp = z0;
Qs = zeros(2,2);
H=zeros(2,3);
Z=zeros(3,3);
G=zeros(3,2);
LandN=0;
CheckL=[];
dmax = 0.25; 

data = flip(noisyRangeData); 
HMT=zeros(N,500);

%% EKF Algorithm
 
for t=1:N
    disp(['t: ', num2str(t), ', Landmarks: ', num2str((size(Zp,1)-3)/2)]);

    Corners = CornerExtractionD(Zp(1:3),t,Threshold,Prominence,data,ang_span);

    if t==1
        %% Initialization after have seen first landmarks
        if ~isempty(Corners)
            for s=1:size(Corners,1)
                CheckL=[CheckL,1];
                Zp(2+2*s) = Corners(s,2)*cos(wrapToPi(Corners(s,3))+Zp(3))+Zp(1);
                Zp(3+2*s) = Corners(s,2)*sin(wrapToPi(Corners(s,3)+Zp(3)))+Zp(2);
                Pp=blkdiag(Pp,eta.*eye(2));
                H=[H,zeros(2,2)];
                Z=blkdiag(Z,eye(2));
                G=[G;zeros(2,2)];
                [Zp,Pp]=CorrectionStep(s,Zp,H,Pp,R,Corners(s,2),Corners(s,3));
                HMT(t,s)=1;
            end%endfor
        end
    else
        %% Computation of distance for each new landmarks wrt old
        HMT(t,1:size(CheckL,2))=HMT(t-1,1:size(CheckL,2));
        %j:iteration on landmarks of new measurement
        for j=1:size(Corners,1)
            distance=zeros(1,size(CheckL,2));
            dis = zeros(1,size(CheckL,2));

            %k:iteration on landmarks of old measurements
            for k=1:size(CheckL,2)
                [dx,dy,q]=differential(Zp,k);

                H=zeros(2,size(H,2));
                H(:,1:3) = [-dx/q -dy/q 0;
                    dy/q^2 -dx/q^2 -1];
                H(:,2+2*k:3+2*k) = [dx/q dy/q; -dy/q^2 dx/q^2];

                delta=[Corners(j,2)-sqrt((Zp(2+2*k)-Zp(1))^2+(Zp(3+2*k)-Zp(2))^2);
                    wrapToPi(Corners(j,3)-atan2(Zp(3+2*k)-Zp(2),(Zp(2+2*k)-Zp(1)))+Zp(3))];

                %Mahalanobis distance
                distance(k)=delta'*inv(H*Pp*H'+R)*delta;
                dis(k) = sqrt((Corners(j, 2) - Zp(2 + 2 *k))^2 + (Corners(j, 3) - Zp(3 + 2 *k))^2);
            end
            %disp(['d: ', num2str(distance)])
            [dstar,k] = min(distance);
            dis_min = min(dis);


            % WORK IN PROGRESS
            if dstar < tau1
                HMT(t,k)=HMT(t,k)+1;
                [Zp, Pp] = CorrectionStep(k, Zp, H, Pp, R, Corners(j, 2), Corners(j, 3));

            elseif dstar > tau2 && dis_min < dmax
                HMT(t,k)=HMT(t,k)+1;
                [Zp, Pp] = CorrectionStep(k, Zp, H, Pp, R, Corners(j, 2), Corners(j, 3));

            elseif dstar > tau2 && dis_min > dmax
                CheckL=[CheckL,1];
                Zp = [Zp; Corners(j, 2) * cos(wrapToPi(Corners(j, 3)) + Zp(3)) + Zp(1); Corners(j, 2) * sin(wrapToPi(Corners(j, 3)) + Zp(3)) + Zp(2)];
                Pp = blkdiag(Pp, eta * eye(2));
                H = [H, zeros(2, 2)];
                Z = blkdiag(Z, eye(2));
                G = [G; zeros(2, 2)];
                [Zp, Pp] = CorrectionStep(size(CheckL,2), Zp, H, Pp, R, Corners(j, 2), Corners(j, 3));
                HMT(t,size(CheckL,2))=HMT(t,size(CheckL,2))+1;
            else
                continue
            end

        end
    end 
%END-DISTANCE--------------------------------------------------------------

    Zc=Zp;
    Pc=Pp;

    % Prediction Step
        % F Jacobian Matrix computation
        F_u = [1  0  -Ts(t)*Uf(t)*sin(Zc(3));
               0  1   Ts(t)*Uf(t)*cos(Zc(3));
               0  0     1];
        Z=blkdiag(F_u,eye(2*size(CheckL,2)));

       %System evalution: robot
       Zp(1) = Zc(1) + Ts(t)*Uf(t)*cos(Zc(3)); 
       Zp(2) = Zc(2) + Ts(t)*Uf(t)*sin(Zc(3));
       Zp(3) = Zc(3) + Ts(t)*Ua(t); 

       %System evalution: landmarks
       for L= 1:size(CheckL,2)
           Zp(2+2*L) = Zc(2+2*L); 
           Zp(3+2*L) = Zc(3+2*L);
       end 

       % G Jacobian Matrix computation
        G_u=[-Ts(t)*cos(Zc(3))  0;
             -Ts(t)*sin(Zc(3))  0;
                0        -Ts(t)]; 
        G = [G_u; zeros(size(CheckL,2)*2,2)]; 

      % Process disturbance evaluation
       if abs(Ua(t)) > wturn
           Qs = Qturn; 
       else
           Qs = Q; 
       end 
       % Computation covariance matrix prediction error
       Pp = Z*Pc*Z'+ G*Qs*G'; 

       % Saving data of interest
        
       if t==1
        LandN=size(CheckL,2);

        Zest(t,:) = Zc'; 
        Pest(t,:) = diag(Pc)'; 
    
       else
        if LandN < size(CheckL,2)
         Zest=[Zest,zeros(t-1,2*(size(CheckL,2)-LandN))];
         Zest(t,:) = Zc';
         Pest=[Pest,zeros(t-1,2*(size(CheckL,2)-LandN))];
         Pest(t,:) = diag(Pc)'; 
         LandN=size(CheckL,2);
 
        else
         Zest(t,:) = Zc'; 
         Pest(t,:) = diag(Pc)'; 
       
       end
       end
       
end
%Trajectory by integration
Zi(1,:)=[41.5,59,pi];

for t=1:N
 Zi(1) = Zi(1) + Ts(t)*Uf(t)*cos(Zi(3)); 
 Zi(2) = Zi(2) + Ts(t)*Uf(t)*sin(Zi(3));
 Zi(3) = Zi(3) + Ts(t)*Ua(t);
 
 Ziest(t,:)=Zi;
end
Ziest=[[41,59,pi];Ziest];

if dataset=="gruppo1_2_1.mat" || dataset=="gruppo1_2_2.mat"
display("Plotting data");
Plot_Data_D;
SLAM_D_PlotAllMap;
display("Done");
else
display("Plotting data");
Plot_Data_D_1SIDE;
display("Done");
end




function [dx,dy,q]=differential(Zp,L)
 dx = Zp(2+2*L)-Zp(1);
 dy = Zp(3+2*L)-Zp(2);
 q = sqrt(dx^2+dy^2);
end

function H=HComputation(dx,dy,q,L,Nland)
    H=zeros(2,3+2*Nland);
    H(:,1:3) = [-dx/q -dy/q 0; 
                dy/q^2 -dx/q^2 -1];
    H(:,2+2*L:3+2*L) = [dx/q dy/q; -dy/q^2 dx/q^2];
end

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

