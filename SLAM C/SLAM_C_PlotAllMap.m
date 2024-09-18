%load("PointsMap1.mat");
%load("PointsMap2.mat")
tend=640;
ra = ang_span / (size(noisyRangeData, 1) - 1);
angles = ang_span / 2 : -ra : -ang_span / 2;
PointsMeasured=[];
for t=1:tend
    parfor i=1:size(noisyRangeData(:, t),1)
        if noisyRangeData(i,t)>0
            [X, Y] = pol2car(noisyRangeData(i, t), angles(i), Pose(t, 1), Pose(t, 2), Pose(t, 3));
            PointsMeasured=[PointsMeasured; [X,Y]];
        end
    end
end

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
 PointsOdometry=[PointsOdometry; [X,Y]];
 end
 end
end


 fig=figure;
    axis equal
    scatter(PointsMeasured(:,1), PointsMeasured(:,2),'.r')
    hold on
    scatter(PointsEstimated(:,1), PointsEstimated(:,2),'.b')
    hold on
    scatter(PointsOdometry(:,1), PointsOdometry(:,2),'.g')
    axis equal
   legend('Map measured','Map estimated','Map odometry')
% 
% figure
% subplot(1,2,1)
% title('Data set 1')
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset;
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left, bottom, ax_width, ax_height];
% axis equal
% %scatter(trueMap(:,1), trueMap(:,2),'.k')
% hold on
% scatter(PointsMeasured(:,1),PointsMeasured(:,2),'.r','LineWidth',0.1)
% scatter(PointsEstimated(:,1), PointsEstimated(:,2),'.b')
% scatter(PointsOdometry(:,1), PointsOdometry(:,2),'.g')
% axis([5 55 15 70])
% legend('Map measured','Map estimated','Map odometry')
% subplot(1,2,2)
% title('Data set 2')
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset;
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left, bottom, ax_width, ax_height];
% axis equal
% %scatter(trueMap(:,1), trueMap(:,2),'.k')
% hold on
% scatter(PointsMeasured2(:,1),PointsMeasured2(:,2),'.r','LineWidth',0.1)
% hold on
% scatter(PointsEstimated2(:,1), PointsEstimated2(:,2),'.b')
% hold on
% scatter(PointsOdometry2(:,1), PointsOdometry2(:,2),'.g')
% axis([0 50 15 70])
% legend('Map measured','Map estimated','Map odometry')



function [X, Y] = pol2car(module, angle, offsetx, offsety, offsetangle)
X = module * cos(wrapToPi(angle) + offsetangle) + offsetx;
Y = module * sin(wrapToPi(angle) + offsetangle) + offsety;
end