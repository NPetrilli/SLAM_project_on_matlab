%%Recovery landmark coordinates
T = TimeStamp;
for i=1:LandN
    marks_test(i,1:2)=[Zest(end,2+2*i) Zest(end,3+2*i)];
end

T = T(1:N); 
subplot(3,4,[1 2])
xlabel('t')
hold on
title('X: Trend') 
plot(T,Zest(:,1),'r',T,Zest(:,1)+3*sqrt(Pest(:,1)),'b--',T,Zest(:,1)-3*sqrt(Pest(:,1)),'b--')
legend("X: Trend","Confidence interval",'Location','southeast')

%Y
subplot(3,4,[5 6])
xlabel('t')
hold on
title('Y: Trend') 
plot(T,Zest(:,2),'r',T,Zest(:,2)+3*sqrt(Pest(:,2)),'b--',T,Zest(:,2)-3*sqrt(Pest(:,2)),'b--')
legend("Y: Trend","Confidence interval",'Location','southeast')

%theta
subplot(3,4,[9 10])
xlabel('t')
hold on
title('Theta: Trend') 
plot(T,Zest(:,3),'r',T,Zest(:,3)+3*sqrt(Pest(:,3)),'b--',T,Zest(:,3)-3*sqrt(Pest(:,3)),'b--')
legend("Theta: Trend","Confidence interval",'Location','southeast')

%Trajectories and landmarks------------------------------------------------
subplot(3,4,[3 4 7 8 11 12])
title('Trajectories and landmarks') 
hold on 
plot(Zest(:,1),Zest(:,2),'b','LineWidth',2)
plot(Ziest(:,1),Ziest(:,2),'-r')
scatter(marks_test(:,1),marks_test(:,2),'.m')
axis([10 47 23 70])
load("trueMap.mat")
plot(trueMap(:,1),trueMap(:,2),'.k')
hold on
for i=1:LandN
    mu=[Zest(end,2+2*i);Zest(end,3+2*i)];
    %Pel=reshape(Ptotal(end,:),[],size(Zest,2))';
    Sigma= Pc(2+2*i:3+2*i,2+2*i:3+2*i);
    p = 0.99; 
    s = -2 * log(1 - p);
    [V, D] = eig(Sigma * s);
    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
    plot(a(1, :) + mu(1), a(2, :) + mu(2),'--g','LineWidth',0.1);
    hold on
end
for i=1:LandN
 text(Zest(end,2+2*i),Zest(end,3+2*i), num2str(HMT(end,i)),'FontSize',5,'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end
legend('Estimated Pose','Odom Pose','Landmarks Pose','Real Map','Confidence Ellipse')
