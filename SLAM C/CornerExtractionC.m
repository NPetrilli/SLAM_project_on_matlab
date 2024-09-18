function Corners = CornerExtractionC(Pose,t,ThresholdNoTurn,ProminenceNoTurn,noisyRangeData,ang_span)

threshold=ThresholdNoTurn;
Prominence=ProminenceNoTurn;

%--------------------------------------------------------------------------
%% Setup angles
ra=ang_span/(size(noisyRangeData,1)-1);
angles=-ang_span/2:ra:ang_span/2;

%--------------------------------------------------------------------------
%% Data & Filtering
[mins,P]=islocalmin(noisyRangeData(:,t),'MinProminence',Prominence);

% Taking measurements (d,a) only of minimum points
% Target = [index of min, distance of min, angle of min]
Target=[find(mins==1),noisyRangeData(mins==1,t),angles(mins==1)',P(mins==1)];

% Filtering by distance [0,4)
DistanceThreshold=100; %NoFilter=high value

Target_filtered = Target((Target(:, 2) >= 0) & (Target(:, 2) < DistanceThreshold), :);

%% STUDY OF CORNER-WALL
[Corners,Wall]=isCorner(Target_filtered,noisyRangeData(:,t),threshold,Pose,t);

end


function [Corners, Wall] = isCorner(Target_filtered, distances, threshold, Pose, t)
Corners = [];
Wall = [];
difference = diff(distances);

for i = 1:size(Target_filtered, 1)
    [Px, Py] = pol2car(Target_filtered(i, 2), Target_filtered(i, 3), Pose(t, 1), Pose(t, 2), Pose(t, 3));

    idx = Target_filtered(i, 1);

    if (idx<35 && Target_filtered(i,4)>1.8) || (idx>145 && Target_filtered(i,4)>1.8)
     tester=((abs(difference(idx - 1) - difference(idx)) / 2)*abs(Target_filtered(i,3)*180/pi))/Target_filtered(i,2);
     threshold=threshold*10;
    end
     tester=((abs(difference(idx - 1) - difference(idx)) / 2)*abs(Target_filtered(i,3)*180/pi))/Target_filtered(i,2);

  
    if tester > threshold
        Corners = [Corners; [Target_filtered(i, 1:end-1), Px, Py, tester, Target_filtered(i, end)]];
        %disp(['t: ', num2str(t), '  CORN', ', test: ', num2str(round(tester,4)), ', ang: ', num2str(Target_filtered(i,3)*180/pi), ', dis: ', num2str(Target_filtered(i,2))]);
    else
        Wall = [Wall; [Target_filtered(i, 1:end-1), Px, Py, tester, Target_filtered(i, end)]];
        %disp(['t: ', num2str(t), '  WALL', ', test: ', num2str(round(tester,4)), ', ang: ', num2str(Target_filtered(i,3)*180/pi), ', dis: ', num2str(Target_filtered(i,2))]);
    end
end
end


function [X,Y]=pol2car(module,angle,offsetx,offsety,offsetangle)
X=module*cos(wrapToPi(angle)+offsetangle)+offsetx;
Y=module*sin(wrapToPi(angle)+offsetangle)+offsety;
end


