% Slam_B-[GANO Group]

%HMT:    "How many time", to count how many landmarks it has seen.
%Zest:   Estimated state of robot's pose and landmarks' positions
%Pest:   Diagonal of Covariance matrix of estimated state
%Ptotal: Covariance matrix of estimated state

%% Loading
clear all 
close all 

%% SETUP DATASET
exercise='B';
dataset="data_point_land_1.mat";

data_number=sscanf(dataset,'data_point_land_%d.mat');
load(dataset);
% Horizon time
T = 1:Ts:(size(Ua)+1)*Ts; 

%% Initialization State 
% Initial robot's state %3x1+2x1
z0 = Pose(1,:)'; 

%% Initialization Covariance Matrix
%Tuning parameter for robot's state
lambda = [0.0001 0.0001 0.0001]';
%Covariance matrix robot's state
P_0 = lambda.*eye(3); %3x3 +eta*diag(eye)2x2
eta=10;

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

tau1=5.9915;
tau2=13.8155;
LandN=0;
HMT=zeros(N,20);
CheckLt=zeros(N,20);
%% EKF Algorithm
for t=1:N
    disp(['t: ', num2str(t), ', Landmarks: ', num2str((size(Zp,1)-3)/2)]);
    if t==1
        %% Initialization after have seen first landmarks
        for s=1:size(Meas.range{t},1)
            CheckL=[CheckL,1];
            Zp(2+2*s) = Meas.range{t}(s)*cos(wrapToPi(Meas.angle{t}(s))+Zp(3))+Zp(1);
            Zp(3+2*s) = Meas.range{t}(s)*sin(wrapToPi(Meas.angle{t}(s)+Zp(3)))+Zp(2);
            Pp=blkdiag(Pp,eta.*eye(2));
            H=[H,zeros(2,2)];
            Z=blkdiag(Z,eye(2));
            G=[G;zeros(2,2)];
            [Zp,Pp]=CorrectionStep(s,Zp,H,Pp,R,Meas.range{t}(s),Meas.angle{t}(s));
            HMT(t,s)=1;
        end
        
    else
        %% Computation of distance for each new landmarks wrt old
         HMT(t,1:size(CheckL,2))=HMT(t-1,1:size(CheckL,2));
        %j:iteration on landmarks of new measurement
        for j=1:size(Meas.range{t},1)
            distance=zeros(1,size(CheckL,2));
            
            %k:iteration on landmarks of old measurements
            for k=1:size(CheckL,2)
                [dx,dy,q]=differential(Zp,k);

                H=zeros(2,size(H,2));
                H(:,1:3) = [-dx/q -dy/q 0;
                    dy/q^2 -dx/q^2 -1];
                H(:,2+2*k:3+2*k) = [dx/q dy/q; -dy/q^2 dx/q^2];
            
                delta=[Meas.range{t}(j)-sqrt((Zp(2+2*k)-Zp(1))^2+(Zp(3+2*k)-Zp(2))^2);
                       wrapToPi(Meas.angle{t}(j)-atan2(Zp(3+2*k)-Zp(2),(Zp(2+2*k)-Zp(1)))+Zp(3))];
                
                %Mahalanobis distance
                distance(k)=delta'*inv(H*Pp*H'+R)*delta;
            end

            [dstar,k] = min(distance);
            
            if dstar < tau1
              HMT(t,k)=HMT(t,k)+1;
              [Zp,Pp]=CorrectionStep(k,Zp,H,Pp,R,Meas.range{t}(j),Meas.angle{t}(j));

            elseif dstar>tau2
                CheckL=[CheckL,1];
                Zp=[Zp;Meas.range{t}(j)*cos(wrapToPi(Meas.angle{t}(j))+Zp(3))+Zp(1);  Meas.range{t}(j)*sin(wrapToPi(Meas.angle{t}(j)+Zp(3)))+Zp(2);];
                Pp=blkdiag(Pp,eta.*eye(2));
                H=[H,zeros(2,2)];
                Z=blkdiag(Z,eye(2));
                G=[G;zeros(2,2)];
                [Zp,Pp]=CorrectionStep(size(CheckL,2),Zp,H,Pp,R,Meas.range{t}(j),Meas.angle{t}(j));
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
        Ptotal(t,:)=reshape(Pc.',1,[]);
        CheckLt(t,:)=[CheckL,zeros(1,size(CheckLt,2)-size(CheckL,2))];
       else
        if LandN < size(CheckL,2)
         Zest=[Zest,zeros(t-1,2*(find(CheckL==1, 1,'last')-LandN))];
         Zest(t,:) = Zc';
         clear Ptotalnew;
         for c=1:t-1
           Pel=reshape(Ptotal(c,:),[],3+LandN*2)';
           Pel=[Pel,zeros(size(Pel,1),2*(size(CheckL,2)-LandN))];
           Pel=[Pel;zeros(2*(size(CheckL,2)-LandN),size(Pel,2))];
           Ptotalnew(c,:)=reshape(Pel.',1,[]);
         end
           Ptotal=Ptotalnew;
           Ptotal(t,:)=reshape(Pc.',1,[]);
           Pest=[Pest,eta*ones(t-1,2*(size(CheckL,2)-LandN))];
           Pest(t,:) = diag(Pc)';
           LandN=size(CheckL,2);
         CheckLt(t,:)=[CheckL,zeros(1,size(CheckLt,2)-size(CheckL,2))];
        else
         Zest(t,:) = Zc'; 
         Ptotal(t,:)=reshape(Pc.',1,[]);
         Pest(t,:) = diag(Pc)'; 
         
         CheckLt(t,:)=[CheckL,zeros(1,size(CheckLt,2)-size(CheckL,2))];
       end
       
       
       end
       
end
%Trajectory by integration
Zi(1,:)=[1,1,0];

for t=1:220
 Zi(1) = Zi(1) + Ts*Uf(t)*cos(Zi(3)); 
 Zi(2) = Zi(2) + Ts*Uf(t)*sin(Zi(3));
 Zi(3) = Zi(3) + Ts*Ua(t);
 
 Ziest(t,:)=Zi;
end

SLAM_B_Data_Plot;

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
