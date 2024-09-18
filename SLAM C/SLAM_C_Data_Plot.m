% Recovery landmark coordinates
T = 1:Ts:(size(Ua))*Ts; 
for i=1:LandN
    marks_test(i,1:2)=[Zest(end,2+2*i) Zest(end,3+2*i)];
end

%X,Y,theta plot in time
figure (1)
clf
sgtitle([exercise,') Dataset',' ', num2str(data_number)])
%X(t)----------------------------------------------------------------------
subplot(3,4,[1 2])

plot(T,Zest(:,1),'b',T,Pose(:,1),'r');
xlabel('t')
title('X(t)') 
hold on
plot(T,Uf(:,1),'g')
legend("X(t) estimated","X(t) real",'Uf(t) command')

%Y(t)----------------------------------------------------------------------
subplot(3,4,[5 6])

plot(T,Zest(:,2),'b',T,Pose(:,2),'r');
xlabel('t')
title('Y(t)') 
hold on
plot(T,Uf(:,1),'g')
legend("Y(t) estimated","Y(t) real",'Uf(t) command')

%Theta(t)------------------------------------------------------------------
subplot(3,4,[9 10])
 
plot(T,Zest(:,3),'b',T,unwrap(Pose(:,3)),'r');
xlabel('t')
title('Theta(t)')
hold on
plot(T,Ua(:,1),'g')
legend("Theta(t) estimated","Theta(t) real",'Ua(t) command')

%Trajectories and landmarks------------------------------------------------
subplot(3,4,[3 4 7 8 11 12])
hold on
title('Trajectories and landmarks') 
plot(Zest(:,1),Zest(:,2),'b','LineWidth',2)

%PlotMapSN(Obstacles)
axis([11 44 22 70])
for i=1:LandN
marks_test(i,1:2)=[Zest(end,2+2*i) Zest(end,3+2*i)];
end
scatter(marks_test(:,1),marks_test(:,2),'*m')
hold on
plot(trueMap(:,1),trueMap(:,2),'.k')
plot([17;Ziest(:,1)],[30;Ziest(:,2)],'-c')
plot(Pose(:,1),Pose(:,2),'g')
for i=1:LandN
 text(Zest(end,2+2*i),Zest(end,3+2*i), num2str(HMT(end,i)),'FontSize',8,'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end
legend('Estimated Pose','Landmarks','Real Map','Odom Pose','Real Pose')

%ERROR FIGURE--------------------------------------------------------------
figure (2)
sgtitle([exercise,') Dataset',' ', num2str(data_number),' ','Errors'])
hold on
%X(t)----------------------------------------------------------------------
subplot(3,4,[1 2])
xlabel('t')
hold on
title('X: Estimation error') 
plot(T,Pose(:,1)-Zest(:,1),'r',T,3*sqrt(Pest(:,1)),'b--',T,-3*sqrt(Pest(:,1)),'b--')
legend("X: Estimation error","Confidence interval")

%Y(t)----------------------------------------------------------------------
subplot(3,4,[5 6])
xlabel('t')
hold on
title('Y: Estimation error') 
plot(T,Pose(:,2)-Zest(:,2),'r',T,3*sqrt(Pest(:,2)),'b--',T,-3*sqrt(Pest(:,2)),'b--')
legend("Y: Estimation error","Confidence interval")
%Theta(t)------------------------------------------------------------------
subplot(3,4,[9 10])
xlabel('t')
hold on
title('Theta: Estimation error') 
plot(T,wrapToPi(Pose(:,3)-Zest(:,3)),'r',T,3*sqrt(Pest(:,3)),'b--',T,-3*sqrt(Pest(:,3)),'b--')
legend("Theta: Estimation error","Confidence interval")



%Trajectories(t)------------------------------------------------------------------
subplot(3,4,[3 4 7 8 11 12])
xlabel('X')
ylabel('Y')
hold on
title('Landmarks estimation error') 
plot(trueMap(:,1),trueMap(:,2),'.k')
hold on
for i=1:LandN
 text(Zest(end,2+2*i),Zest(end,3+2*i), num2str(HMT(end,i)),'FontSize',8,'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end
axis([11 44 22 70])
for i=1:LandN
    mu=[Zest(end,2+2*i);Zest(end,3+2*i)];
    %Pel=reshape(Ptotal(end,:),[],size(Zest,2))';
    Sigma= Pc(2+2*i:3+2*i,2+2*i:3+2*i);
    p = 0.99; 
    s = -2 * log(1 - p);
    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    plot(a(1, :) + mu(1), a(2, :) + mu(2),'r');
   
    %scatter(Landmarks(i,1),Landmarks(i,2),'g.');
    hold on
    scatter(mu(1),mu(2),'b*');
    %pause(2)
end
legend("Real map","Confidence ellipse","Estimated landmarks")




