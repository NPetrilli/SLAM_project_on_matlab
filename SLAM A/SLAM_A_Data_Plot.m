% Recovery landmark coordinates
for i=1:20
    marks_test(i,1:2)=[Zest(end,2+2*i) Zest(end,3+2*i)];
end

%X,Y,theta plot in time
figure (1)
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
 
plot(T,Zest(:,3),'b',T,Pose(:,3),'r');
xlabel('t')
title('Theta(t)')
hold on
plot(T,Ua(:,1),'g')
legend("Theta(t) estimated","Theta(t) real",'Ua(t) command','Location','northwest')

%Trajectories and landmarks------------------------------------------------
subplot(3,4,[3 4 7 8 11 12])
title('Trajectories and landmarks') 
hold on
plot(Pose(:,1),Pose(:,2),'r')
plot(Zest(:,1),Zest(:,2),'b')
plot([1;Ziest(:,1)],[1;Ziest(:,2)])
xlabel('x')
ylabel('y')
scatter(Landmarks(:,1),Landmarks(:,2),'r')
scatter(marks_test(:,1),marks_test(:,2),'b')
for i=1:20
 text(Zest(end,2+2*i),Zest(end,3+2*i), num2str(HMT(end,i)),'FontSize',8,'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end
legend("Real Pose","Estimated Pose","Traj Integrated","Real Landmarks","Estimated Landmarks")


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
%plot(T,Ua(:,1),'g')
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
plot(T,Pose(:,3)-Zest(:,3),'r',T,3*sqrt(Pest(:,3)),'b--',T,-3*sqrt(Pest(:,3)),'b--')
legend("Theta: Estimation error","Confidence interval")



%Trajectories(t)------------------------------------------------------------------
subplot(3,4,[3 4 7 8 11 12])
xlabel('X')
ylabel('Y')
hold on
title('Landmarks estimation error') 
parfor i=1:20
    mu=[Zest(end,2+2*i);Zest(end,3+2*i)];
    Pel=reshape(Ptotal(end,:),[],43)';
    Sigma= Pel(2+2*i:3+2*i,2+2*i:3+2*i);
    p = 0.99; 
    s = -2 * log(1 - p);
    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

    plot(a(1, :) + mu(1), a(2, :) + mu(2),'r');
    hold on
    scatter(Landmarks(i,1),Landmarks(i,2),'.',"MarkerEdgeColor","#D95319");
    scatter(mu(1),mu(2),'b*');
end
parfor i=1:20
 text(Zest(end,2+2*i),Zest(end,3+2*i), num2str(HMT(end,i)),'FontSize',8,'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end
legend("Confidence ellipse","Real landmarks","Estimated landmarks")




