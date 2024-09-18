% Slam_C-[GANO Group]

%HMT:    "How many time", to count how many landmarks it has seen.
%Zest:   Estimated state of robot's pose and landmarks' positions
%Pest:   Diagonal of Covariance matrix of estimated state
%Ptotal: Covariance matrix of estimated state

%% Loading
clear all 
close all 

%% SETUP DATASET
exercise='C';
dataset="data_sim_lidar_1.mat";

data_number=sscanf(dataset,'data_sim_lidar_%d.mat');
load(dataset);

% Horizon time
T = 1:Ts:(size(Ua)+1)*Ts; 

%% Initialization State 
% Initial robot's state %3x1+2x1
z0 = Pose(1,:)'; 

%% Initialization Covariance Matrix
%Tuning parameter for robot's state
lambda = [0.002 0.002 0.00012]';
%Covariance matrix robot's state
P_0 = lambda.*eye(3); %3x3 +eta*diag(eye)2x2
eta=15;

%--------------------------------------------------------------------------


%% Auxiliary parameters for EKF Algorithm
N = size(Ua,1); 
Pp = P_0; 
Zp = z0;
Qs = zeros(2,2);
H=zeros(2,3);
Z=zeros(3,3);
G=zeros(3,2);
CheckL=[];
LandN=0;

if dataset=="data_sim_lidar_1.mat"
    % %Parameters CornerAnalyzer DAtaset1
    tau1=5.9;
    tau2=21.507;
    ThresholdTurn=0.035;
    ProminenceTurn=0.250;
    ThresholdNoTurn=0.21;
    ProminenceNoTurn=0.1110;
    phi_th=0.03;
else
    %Parameters CornerAnalyzer DatasetS2
    tau1=3.85;
    tau2=19.2;
    ThresholdTurn=0.035;
    ProminenceTurn=0.250;
    ThresholdNoTurn=0.19;
    ProminenceNoTurn=0.113;
    phi_th=0.03;
end
data = flip(noisyRangeData); 
%How many times it sees the landmarks
HMT=zeros(N,500);
CheckLt=zeros(N,500);
%% EKF Algorithm
for t=1:N
    disp(['t: ', num2str(t), ', Landmarks: ', num2str((size(Zp,1)-3)/2)]);
    
    Corners = CornerExtractionC(Pose,t,ThresholdNoTurn,ProminenceNoTurn,data,ang_span);

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
        end
       end
    else
        %% Computation of distance for each new landmarks wrt old
        HMT(t,1:size(CheckL,2))=HMT(t-1,1:size(CheckL,2));
        %j:iteration on landmarks of new measurement
        for j=1:size(Corners,1)
            distance=zeros(1,size(CheckL,2));
            disPhis=zeros(1,size(CheckL,2));
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

                [Lx,Ly]=pol2car(Corners(j,2),Corners(j,3),Zp(1),Zp(2),Zp(3));
                dis(k)=sqrt((Lx-Zp(2+2*k))^2+(Ly-Zp(3+2*k))^2);
            end
            %disp(['d: ', num2str(distance)])
            [dstar,k] = min(distance);
            phi_min=min(dis);
    
           
            if dstar < tau1
              HMT(t,k)=HMT(t,k)+1;
              [Zp,Pp]=CorrectionStep(k,Zp,H,Pp,R,Corners(j,2),Corners(j,3));
            elseif dstar>tau2 && phi_min<phi_th
              [Zp,Pp]=CorrectionStep(k,Zp,H,Pp,R,Corners(j,2),Corners(j,3));
            elseif dstar>tau2
                CheckL=[CheckL,1];
                Zp=[Zp;Corners(j,2)*cos(wrapToPi(Corners(j,3))+Zp(3))+Zp(1);  Corners(j,2)*sin(wrapToPi(Corners(j,3)+Zp(3)))+Zp(2);];
                Pp=blkdiag(Pp,eta.*eye(2));
                H=[H,zeros(2,2)];
                Z=blkdiag(Z,eye(2));
                G=[G;zeros(2,2)];
                [Zp,Pp]=CorrectionStep(size(CheckL,2),Zp,H,Pp,R,Corners(j,2),Corners(j,3));
                HMT(t,size(CheckL,2))=HMT(t,size(CheckL,2))+1;
            else
                continue;
            end 

        end
    end 
%END-DISTANCE--------------------------------------------------------------

    Zc=Zp;
    Pc=Pp;

    % Prediction Step
        % F Jacobian Matrix computation
        F_u = [1  0  -Ts*Uf(t)*sin(Zc(3));
               0  1   Ts*Uf(t)*cos(Zc(3));
               0  0     1];
        Z=blkdiag(F_u,eye(2*size(CheckL,2)));

       %System evalution: robot
       Zp(1) = Zc(1) + Ts*Uf(t)*cos(Zc(3)); 
       Zp(2) = Zc(2) + Ts*Uf(t)*sin(Zc(3));
       Zp(3) = Zc(3) + Ts*Ua(t); 

       %System evalution: landmarks
       for L= 1:size(CheckL,2)
           Zp(2+2*L) = Zc(2+2*L); 
           Zp(3+2*L) = Zc(3+2*L);
       end 

       % G Jacobian Matrix computation
        G_u=[-Ts*cos(Zc(3))  0;
             -Ts*sin(Zc(3))  0;
                0        -Ts]; 
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
Zi(1,:)=[17,30,0];

for t=1:N
 Zi(1) = Zi(1) + Ts*Uf(t)*cos(Zi(3)); 
 Zi(2) = Zi(2) + Ts*Uf(t)*sin(Zi(3));
 Zi(3) = Zi(3) + Ts*Ua(t);
 
 Ziest(t,:)=Zi;
end
Ziest=[[17,30,0];Ziest];
display("Plotting data");
SLAM_C_Data_Plot;
SLAM_C_PlotAllMap
display("Done");

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

function [X,Y]=pol2car(module,angle,offsetx,offsety,offsetangle)
X=module*cos(wrapToPi(angle)+offsetangle)+offsetx;
Y=module*sin(wrapToPi(angle)+offsetangle)+offsety;
end
