%Plot all map
tend=N;
ra = ang_span / (size(noisyRangeData, 1) - 1);
angles = ang_span / 2 : -ra : -ang_span / 2;
PointsEstimated=[];
for t=1:tend
 parfor i=1:size(noisyRangeData(:, t),1)
 if noisyRangeData(i,t)>0
 [X, Y] = pol2car(noisyRangeData(i, t), angles(i), Zest(t, 1), Zest(t, 2), Zest(t, 3));
 PointsEstimated=[PointsEstimated; [X,Y]];
 end
 end
end
PointsOdometry=[]; 
for t=1:tend
 parfor i=1:size(noisyRangeData(:, t),1)
 if noisyRangeData(i,t)>0    
 [X, Y] = pol2car(noisyRangeData(i, t), angles(i), Ziest(t, 1), Ziest(t, 2), Ziest(t, 3));
 PointsOdometry=[PointsOdometry;[X,Y]];
 end
 end
end
if dataset=="gruppo1_2_1.mat" || dataset=="gruppo1_2_2.mat"
    fig=figure;
    axis equal
    hold on
    scatter(PointsEstimated(:,1), PointsEstimated(:,2),'.b')
    hold on
    scatter(PointsOdometry(:,1), PointsOdometry(:,2),'.r')
    plot(Zest(:,1),Zest(:,2),'m','LineWidth',1.5)
    plot(Ziest(:,1),Ziest(:,2),'-g')
    legend('Map estimated','Map odometry')
end



function [X, Y] = pol2car(module, angle, offsetx, offsety, offsetangle)
X = module * cos(wrapToPi(angle) + offsetangle) + offsetx;
Y = module * sin(wrapToPi(angle) + offsetangle) + offsety;
end